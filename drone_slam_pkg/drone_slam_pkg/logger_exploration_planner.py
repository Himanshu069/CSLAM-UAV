#!/usr/bin/env python3
"""
EXPLORATION METRICS LOGGER NODE
Subscribes to all metrics/ topics published by AutonomousExplorer
and writes them to structured CSV files + a final summary JSON.

Output structure (one folder per run):
  logs/run_<timestamp>/
    coverage.csv          — (t, coverage_%, explored_area_m2, coverage_rate, path_efficiency)
    path.csv              — (t, path_length_m, avg_speed)
    planning.csv          — (t, rrt_solve_ms, rrt_iterations, rrt_failed, frontier_count)
    safety.csv            — (t, clearance_m, min_clearance_running, hard_brake_count, force_ratio)
    events.csv            — (t, event_type)   [stuck, escape, replan]
    apf_internal.csv      — (t, local_min_count)
    summary.json          — aggregated stats written on shutdown
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32, Int32, Bool
import statistics

import os
import csv
import json
from datetime import datetime


class ExplorationLogger(Node):
    def __init__(self):
        super().__init__("exploration_logger")

        self.declare_parameter("log_dir", "logs")
        self.declare_parameter("drone_namespace", "")   # e.g. "drone_0" or ""

        log_base   = self.get_parameter("log_dir").value
        self.ns    = self.get_parameter("drone_namespace").value  # prefix for multi-drone

        timestamp  = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_name   = f"run_{timestamp}" if not self.ns else f"run_{self.ns}_{timestamp}"
        self.run_dir = os.path.join(log_base, run_name)
        os.makedirs(self.run_dir, exist_ok=True)
        self.get_logger().info(f"Logging to: {self.run_dir}")

        self._files   = {}
        self._writers = {}

        self._open_csv("coverage",    ["t", "coverage_pct", "explored_area_m2",
                                       "coverage_rate_m2s", "path_efficiency_m2pm"])
        self._open_csv("path",        ["t", "path_length_m", "avg_speed_ms"])
        self._open_csv("planning",    ["t", "rrt_solve_ms", "rrt_iterations",
                                       "rrt_failed", "frontier_count"])
        self._open_csv("safety",      ["t", "clearance_m", "min_clearance_running_m",
                                        "apf_force_ratio","tracking_error_m"])
        self._open_csv("cbf",          ["t", "h1", "h2", "phi1_m", "phi2_m",
                                        "slack", "cbf1_active", "cbf2_active",
                                        "speed_clipped", "soft_delta"])
        self._open_csv("events",      ["t", "event_type"])
        self._open_csv("apf_internal",["t", "local_min_count"])

        self._start_time         = self.get_clock().now()

        # coverage
        self._last_coverage_pct  = 0.0
        self._last_area          = 0.0
        self._last_rate          = 0.0
        self._last_efficiency    = 0.0
        self._coverage_samples   = []          # list of (t, pct) for curve

        # path
        self._last_path_length   = 0.0
        self._last_avg_speed     = 0.0

        # planning
        self._rrt_times          = []          # ms per call
        self._rrt_iters          = []
        self._rrt_fail_count     = 0
        self._last_frontier_count= 0

        # safety
        self._last_clearance     = float('inf')
        self._global_min_clear   = float('inf')
        self._last_force_ratio   = 0.0
        self._force_ratio_samples= []
        self._last_tracking_error = 0.0
        self._tracking_error_samples = [] 

        # ── CBF state ──
        self._last_h1            = 0.0
        self._last_h2            = 0.0
        self._last_phi1          = 0.0
        self._last_phi2          = 0.0
        self._last_cbf_slack     = 0.0
        self._last_cbf1_active   = False
        self._last_cbf2_active   = False
        self._last_speed_clipped = False
        self._last_soft_delta    = 0.0
        self._cbf_infeasible_count = 0
 
        # CBF summary samples
        self._cbf_h1_samples     = []
        self._cbf_h2_samples     = []
        self._cbf_slack_samples  = []
        self._cbf_phi1_samples   = []
        self._cbf_phi2_samples   = []
        self._cbf1_active_count  = 0   # ticks where c1 fired
        self._cbf2_active_count  = 0   # ticks where c2 fired
        self._cbf_speed_clip_count = 0
        self._cbf_total_ticks    = 0  

        # events
        self._stuck_count        = 0
        self._escape_count       = 0
        self._replan_count       = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        def topic(name):
            return f"/{self.ns}/{name}" if self.ns else f"/{name}"

        # Coverage
        self.create_subscription(Float32, topic("metrics/coverage_percent"),
                                 self._cb_coverage_pct,   qos)
        self.create_subscription(Float32, topic("metrics/explored_area_m2"),
                                 self._cb_area,           qos)
        self.create_subscription(Float32, topic("metrics/coverage_rate"),
                                 self._cb_coverage_rate,  qos)
        self.create_subscription(Float32, topic("metrics/path_efficiency"),
                                 self._cb_efficiency,     qos)
        
        self.create_subscription(Float32, topic("metrics/path_tracking_error_m"),
                         self._cb_tracking_error, qos)
        
        # Path
        self.create_subscription(Float32, topic("metrics/path_length_m"),
                                 self._cb_path_length,    qos)
        self.create_subscription(Float32, topic("metrics/avg_speed"),
                                 self._cb_avg_speed,      qos)

        # Planning
        self.create_subscription(Float32, topic("metrics/rrt_solve_time_ms"),
                                 self._cb_rrt_time,       qos)
        self.create_subscription(Int32,   topic("metrics/rrt_iterations_used"),
                                 self._cb_rrt_iters,      qos)
        self.create_subscription(Bool,    topic("metrics/rrt_failed"),
                                 self._cb_rrt_fail,       qos)
        self.create_subscription(Int32,   topic("metrics/frontier_count"),
                                 self._cb_frontier_count, qos)

        # Safety
        self.create_subscription(Float32, topic("metrics/obstacle_clearance_m"),
                                 self._cb_clearance,      qos)
        self.create_subscription(Float32, topic("metrics/min_clearance_running"),
                                 self._cb_min_clearance,  qos)
        self.create_subscription(Float32, topic("metrics/apf_force_ratio"),
                                 self._cb_force_ratio,    qos)
        
        self.create_subscription(Float32, topic("metrics/cbf_h1"),
                                  self._cb_cbf_h1,             qos)
        self.create_subscription(Float32, topic("metrics/cbf_h2"),
                                  self._cb_cbf_h2,             qos)
        self.create_subscription(Float32, topic("metrics/cbf_phi1_m"),
                                  self._cb_cbf_phi1,           qos)
        self.create_subscription(Float32, topic("metrics/cbf_phi2_m"),
                                  self._cb_cbf_phi2,           qos)
        self.create_subscription(Float32, topic("metrics/cbf_slack"),
                                  self._cb_cbf_slack,          qos)
        self.create_subscription(Bool,    topic("metrics/cbf1_obstacle_active"),
                                  self._cb_cbf1_active,        qos)
        self.create_subscription(Bool,    topic("metrics/cbf2_frontier_active"),
                                  self._cb_cbf2_active,        qos)
        self.create_subscription(Int32,   topic("metrics/cbf_infeasible_count"),
                                  self._cb_cbf_infeasible,     qos)
        self.create_subscription(Float32, topic("metrics/cbf_soft_delta"),
                                  self._cb_cbf_delta,          qos)
        self.create_subscription(Bool,    topic("metrics/cbf_speed_clipped"),
                                  self._cb_cbf_speed_clipped,  qos)

        # Events
        self.create_subscription(Bool,    topic("metrics/stuck_event"),
                                 self._cb_stuck,          qos)
        self.create_subscription(Bool,    topic("metrics/escape_triggered"),
                                 self._cb_escape,         qos)
        self.create_subscription(Bool,    topic("metrics/rrt_replan_triggered"),
                                 self._cb_replan,         qos)

        # APF internal
        self.create_subscription(Int32,   topic("metrics/local_min_count"),
                                 self._cb_local_min,      qos)

        # periodic flush timer — write buffered rows every 5 s
        self._row_buffer = {k: [] for k in
                            ["coverage", "path", "planning",
                             "safety", "cbf","events", "apf_internal"]}
        
        self.create_timer(1.0,  self._write_coverage_row)
        self.create_timer(0.1,  self._write_safety_row)
        self.create_timer(0.1,  self._write_cbf_row) 
        self.create_timer(1.0, self._flush_buffers)

        self.get_logger().info("Exploration Logger ONLINE")
        self.get_logger().info(f"  Namespace : '{self.ns}' (empty = single drone)")
        self.get_logger().info(f"  Run dir   : {self.run_dir}")



    def _open_csv(self, name: str, headers: list):
        path = os.path.join(self.run_dir, f"{name}.csv")
        f = open(path, "w", newline="")
        w = csv.DictWriter(f, fieldnames=headers)
        w.writeheader()
        self._files[name]   = f
        self._writers[name] = w

    def _buf(self, sheet: str, row: dict):
        """Buffer a row — flushed every 5 s to avoid hammering disk."""
        self._row_buffer[sheet].append(row)

    def _flush_buffers(self):
        for sheet, rows in self._row_buffer.items():
            if rows:
                for row in rows:
                    self._writers[sheet].writerow(row)
                self._files[sheet].flush()
                self._row_buffer[sheet] = []

    def _t(self) -> float:
        """Elapsed seconds since node start."""
        return (self.get_clock().now() - self._start_time).nanoseconds / 1e9

    def _cb_tracking_error(self, msg: Float32):
        self._last_tracking_error = msg.data
        self._tracking_error_samples.append(msg.data)

    def _cb_coverage_pct(self, msg: Float32):
        self._last_coverage_pct = msg.data
        # self._coverage_samples.append((self._t(), msg.data))  # keep this for summary

    def _cb_area(self, msg: Float32):
        self._last_area = msg.data

    def _cb_coverage_rate(self, msg: Float32):
        self._last_rate = msg.data

    def _cb_efficiency(self, msg: Float32):
        self._last_efficiency = msg.data

    def _write_coverage_row(self):
        t = self._t()
        self._coverage_samples.append((t, self._last_coverage_pct))
        self._buf("coverage", {
            "t":                    round(t, 3),
            "coverage_pct":         round(self._last_coverage_pct, 4),
            "explored_area_m2":     round(self._last_area, 4),
            "coverage_rate_m2s":    round(self._last_rate, 6),
            "path_efficiency_m2pm": round(self._last_efficiency, 6),
        })


    def _cb_path_length(self, msg: Float32):
        self._last_path_length = msg.data
        self._buf("path", {
            "t":              round(self._t(), 3),
            "path_length_m":  round(msg.data, 4),
            "avg_speed_ms":   round(self._last_avg_speed, 4),
        })

    def _cb_avg_speed(self, msg: Float32):
        self._last_avg_speed = msg.data



    def _cb_rrt_time(self, msg: Float32):
        self._rrt_times.append(msg.data)
        self._buf("planning", {
            "t":               round(self._t(), 3),
            "rrt_solve_ms":    round(msg.data, 2),
            "rrt_iterations":  self._rrt_iters[-1] if self._rrt_iters else "",
            "rrt_failed":      0,
            "frontier_count":  self._last_frontier_count,
        })

    def _cb_rrt_iters(self, msg: Int32):
        self._rrt_iters.append(msg.data)

    def _cb_rrt_fail(self, msg: Bool):
        if msg.data:
            self._rrt_fail_count += 1
            self._buf("planning", {
                "t":               round(self._t(), 3),
                "rrt_solve_ms":    "",
                "rrt_iterations":  "",
                "rrt_failed":      1,
                "frontier_count":  self._last_frontier_count,
            })

    def _cb_frontier_count(self, msg: Int32):
        self._last_frontier_count = msg.data
        self._buf("planning", {
            "t":               round(self._t(), 3),
            "rrt_solve_ms":    "",
            "rrt_iterations":  "",
            "rrt_failed":      0,
            "frontier_count":  msg.data,
        })


    def _cb_clearance(self, msg: Float32):
        self._last_clearance = msg.data

    def _cb_min_clearance(self, msg: Float32):
        self._global_min_clear = min(self._global_min_clear, msg.data)

    def _cb_force_ratio(self, msg: Float32):
        self._last_force_ratio = msg.data
        self._force_ratio_samples.append(msg.data)

    def _write_safety_row(self):
        self._buf("safety", {
            "t":                        round(self._t(), 3),
            "clearance_m":              round(self._last_clearance, 4),
            "min_clearance_running_m":  round(self._global_min_clear, 4)
                                        if self._global_min_clear != float('inf') else "",
            "apf_force_ratio":          round(self._last_force_ratio, 4),
            "tracking_error_m":         round(self._last_tracking_error, 4),
        })

    def _cb_cbf_h1(self, msg: Float32):
        self._last_h1 = msg.data
        self._cbf_h1_samples.append(msg.data)
 
    def _cb_cbf_h2(self, msg: Float32):
        self._last_h2 = msg.data
        self._cbf_h2_samples.append(msg.data)
 
    def _cb_cbf_phi1(self, msg: Float32):
        self._last_phi1 = msg.data
        self._cbf_phi1_samples.append(msg.data)
 
    def _cb_cbf_phi2(self, msg: Float32):
        self._last_phi2 = msg.data
        self._cbf_phi2_samples.append(msg.data)
 
    def _cb_cbf_slack(self, msg: Float32):
        self._last_cbf_slack = msg.data
        self._cbf_slack_samples.append(msg.data)
 
    def _cb_cbf1_active(self, msg: Bool):
        self._last_cbf1_active = msg.data
        if msg.data:
            self._cbf1_active_count += 1
        self._cbf_total_ticks += 1
 
    def _cb_cbf2_active(self, msg: Bool):
        self._last_cbf2_active = msg.data
        if msg.data:
            self._cbf2_active_count += 1
 
    def _cb_cbf_infeasible(self, msg: Int32):
        # Only log an event when the count increments
        if msg.data > self._cbf_infeasible_count:
            self._buf("events", {
                "t":          round(self._t(), 3),
                "event_type": "cbf_infeasible",
            })
        self._cbf_infeasible_count = msg.data
 
    def _cb_cbf_delta(self, msg: Float32):
        self._last_soft_delta = msg.data
 
    def _cb_cbf_speed_clipped(self, msg: Bool):
        self._last_speed_clipped = msg.data
        if msg.data:
            self._cbf_speed_clip_count += 1
            self._buf("events", {
                "t":          round(self._t(), 3),
                "event_type": "cbf_speed_clipped",
            })
 
    def _write_cbf_row(self):
        """10 Hz snapshot of continuous CBF state."""
        self._buf("cbf", {
            "t":            round(self._t(), 3),
            "h1":           round(self._last_h1,  4),
            "h2":           round(self._last_h2,  4),
            "phi1_m":       round(self._last_phi1, 4),
            "phi2_m":       round(self._last_phi2, 4),
            "slack":        round(self._last_cbf_slack, 5),
            "cbf1_active":  int(self._last_cbf1_active),
            "cbf2_active":  int(self._last_cbf2_active),
            "speed_clipped":int(self._last_speed_clipped),
            "soft_delta":   round(self._last_soft_delta, 6),
        })
 

    def _cb_stuck(self, msg: Bool):
        if msg.data:
            self._stuck_count += 1
            self._buf("events", {"t": round(self._t(), 3), "event_type": "stuck"})

    def _cb_escape(self, msg: Bool):
        if msg.data:
            self._escape_count += 1
            self._buf("events", {"t": round(self._t(), 3), "event_type": "escape"})

    def _cb_replan(self, msg: Bool):
        if msg.data:
            self._replan_count += 1
            self._buf("events", {"t": round(self._t(), 3), "event_type": "replan"})



    def _cb_local_min(self, msg: Int32):
        self._buf("apf_internal", {
            "t":               round(self._t(), 3),
            "local_min_count": msg.data,
        })



    def _write_summary(self):
        total_time = self._t()

        # coverage curve — time to reach 25 / 50 / 75 / 90 % 
        def time_to_pct(target):
            for t, pct in self._coverage_samples:
                if pct >= target:
                    return round(t, 2)
            return None

        # RRT stats
        avg_rrt_ms  = round(sum(self._rrt_times) / len(self._rrt_times), 2) \
                      if self._rrt_times else None
        med_rrt_ms = round(statistics.median(self._rrt_times), 2) if self._rrt_times else None
        avg_iters   = round(sum(self._rrt_iters) / len(self._rrt_iters), 1) \
                      if self._rrt_iters else None

        # force ratio stats
        avg_ratio   = round(sum(self._force_ratio_samples) /
                            len(self._force_ratio_samples), 4) \
                      if self._force_ratio_samples else None
        max_ratio   = round(max(self._force_ratio_samples), 4) \
                      if self._force_ratio_samples else None
        
        avg_tracking = round(sum(self._tracking_error_samples) / 
                     len(self._tracking_error_samples), 4) \
               if self._tracking_error_samples else None
        max_tracking = round(max(self._tracking_error_samples), 4) \
               if self._tracking_error_samples else None
        
        def safe_avg(lst):
            return round(sum(lst) / len(lst), 4) if lst else None
 
        # def safe_med(lst):
        #     return round(statistics.median(lst), 4) if lst else None
 
        def safe_min(lst):
            return round(min(lst), 4) if lst else None
        
        cbf_intervention_rate = round(
            (self._cbf1_active_count + self._cbf2_active_count) /
            max(self._cbf_total_ticks * 2, 1), 4
        ) if self._cbf_total_ticks else None

        summary = {
            "run_dir":                  self.run_dir,
            "drone_namespace":          self.ns,
            "total_exploration_time_s": round(total_time, 2),

            # ── coverage ──
            "final_coverage_pct":       round(self._last_coverage_pct, 2),
            "final_explored_area_m2":   round(self._last_area, 3),
            "time_to_25pct_s":          time_to_pct(25.0),
            "time_to_50pct_s":          time_to_pct(50.0),
            "time_to_75pct_s":          time_to_pct(75.0),
            "time_to_90pct_s":          time_to_pct(90.0),

            # ── path ──
            "total_path_length_m":      round(self._last_path_length, 3),
            "avg_speed_ms":             round(self._last_avg_speed, 4),
            "final_path_efficiency":    round(self._last_efficiency, 4),

            # ── planning ──
            "total_replans":            self._replan_count,
            "total_rrt_failures":       self._rrt_fail_count,
            "avg_rrt_solve_ms":         avg_rrt_ms,
            "median_rrt_solve_ms":      med_rrt_ms,
            "avg_rrt_iterations":       avg_iters,
            "rrt_success_rate_pct":     round(
                100.0 * len(self._rrt_times) /
                max(len(self._rrt_times) + self._rrt_fail_count, 1), 2),

            # ── safety ──
            "global_min_clearance_m":   round(self._global_min_clear, 4)
                                        if self._global_min_clear != float('inf') else None,
            "avg_apf_force_ratio":      avg_ratio,
            "max_apf_force_ratio":      max_ratio,
            "avg_tracking_error_m":     avg_tracking,
            "max_tracking_error_m":     max_tracking,

            #CBF
            "cbf_total_ticks_evaluated":   self._cbf_total_ticks,
            "cbf1_obstacle_active_ticks":  self._cbf1_active_count,
            "cbf2_frontier_active_ticks":  self._cbf2_active_count,
            "cbf_intervention_rate":       cbf_intervention_rate,
            "cbf_infeasible_total":        self._cbf_infeasible_count,
            "cbf_speed_clip_total":        self._cbf_speed_clip_count,
            "avg_h1":                      safe_avg(self._cbf_h1_samples),
            "min_h1":                      safe_min(self._cbf_h1_samples),
            "avg_h2":                      safe_avg(self._cbf_h2_samples),
            "min_h2":                      safe_min(self._cbf_h2_samples),
            "avg_phi1_m":                  safe_avg(self._cbf_phi1_samples),
            "min_phi1_m":                  safe_min(self._cbf_phi1_samples),
            "avg_cbf_slack":               safe_avg(self._cbf_slack_samples),
            "max_cbf_slack":               round(max(self._cbf_slack_samples), 5)
                                           if self._cbf_slack_samples else None,
 
            # ── events ──
            "total_stuck_events":       self._stuck_count,
            "total_escape_events":      self._escape_count,
        }

        path = os.path.join(self.run_dir, "summary.json")
        with open(path, "w") as f:
            json.dump(summary, f, indent=2)

        self.get_logger().info("=" * 50)
        self.get_logger().info("LOGGER SUMMARY")
        self.get_logger().info(f"  Total time       : {round(total_time, 1)} s")
        self.get_logger().info(f"  Final coverage   : {round(self._last_coverage_pct, 1)} %")
        self.get_logger().info(f"  Path length      : {round(self._last_path_length, 2)} m")
        self.get_logger().info(f"  RRT* avg solve   : {avg_rrt_ms} ms")
        self.get_logger().info(f"  Stuck events     : {self._stuck_count}")
        min_str = f"{round(self._global_min_clear, 3)} m" if self._global_min_clear != float('inf') else "N/A"
        self.get_logger().info(f"  Min clearance    : {min_str}")
        self.get_logger().info(f"  CBF ticks          : {self._cbf_total_ticks}")
        self.get_logger().info(f"  CBF c1 active      : {self._cbf1_active_count} ticks")
        self.get_logger().info(f"  CBF c2 active      : {self._cbf2_active_count} ticks")
        self.get_logger().info(f"  CBF intervention   : {cbf_intervention_rate}")
        self.get_logger().info(f"  CBF infeasible     : {self._cbf_infeasible_count}")
        self.get_logger().info(f"  CBF speed clips    : {self._cbf_speed_clip_count}")
        h1_min = safe_min(self._cbf_h1_samples)
        self.get_logger().info(f"  min h1 (obstacle)  : {h1_min}")
        h2_min = safe_min(self._cbf_h2_samples)
        self.get_logger().info(f"  min h2 (frontier)  : {h2_min}")
        track_str = f"{avg_tracking} m (max: {max_tracking} m)" if avg_tracking else "N/A"
        self.get_logger().info(f"  Tracking error   : {track_str}")
        self.get_logger().info(f"  Summary written  : {path}")
        self.get_logger().info("=" * 50)

    def destroy_node(self):
        """Called on Ctrl-C / shutdown — flush and write summary."""
        self._flush_buffers()
        self._write_summary()
        for f in self._files.values():
            f.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = ExplorationLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()