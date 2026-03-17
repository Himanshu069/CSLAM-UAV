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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float32, Int32, Bool
import statistics

import os
import csv
import json
import math
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
                                       "hard_brake_count", "apf_force_ratio"])
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
        self._last_hard_brakes   = 0
        self._last_force_ratio   = 0.0
        self._force_ratio_samples= []

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

        self.create_timer(1.0, self._write_coverage_row)
        self.create_timer(0.1, self._write_safety_row)
        # Coverage
        self.create_subscription(Float32, topic("metrics/coverage_percent"),
                                 self._cb_coverage_pct,   qos)
        self.create_subscription(Float32, topic("metrics/explored_area_m2"),
                                 self._cb_area,           qos)
        self.create_subscription(Float32, topic("metrics/coverage_rate"),
                                 self._cb_coverage_rate,  qos)
        self.create_subscription(Float32, topic("metrics/path_efficiency"),
                                 self._cb_efficiency,     qos)

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
        self.create_subscription(Int32,   topic("metrics/hard_brake_count"),
                                 self._cb_hard_brake,     qos)
        self.create_subscription(Float32, topic("metrics/apf_force_ratio"),
                                 self._cb_force_ratio,    qos)

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
                             "safety", "events", "apf_internal"]}
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

    def _cb_hard_brake(self, msg: Int32):
        self._last_hard_brakes = msg.data

    def _cb_force_ratio(self, msg: Float32):
        self._last_force_ratio = msg.data
        self._force_ratio_samples.append(msg.data)

    def _write_safety_row(self):
        self._buf("safety", {
            "t":                        round(self._t(), 3),
            "clearance_m":              round(self._last_clearance, 4),
            "min_clearance_running_m":  round(self._global_min_clear, 4)
                                        if self._global_min_clear != float('inf') else "",
            "hard_brake_count":         self._last_hard_brakes,
            "apf_force_ratio":          round(self._last_force_ratio, 4),
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
            "total_hard_brakes":        self._last_hard_brakes,
            "avg_apf_force_ratio":      avg_ratio,
            "max_apf_force_ratio":      max_ratio,

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
        self.get_logger().info(f"  Hard brakes      : {self._last_hard_brakes}")
        self.get_logger().info(f"  Stuck events     : {self._stuck_count}")
        min_str = f"{round(self._global_min_clear, 3)} m" if self._global_min_clear != float('inf') else "N/A"
        self.get_logger().info(f"  Min clearance    : {min_str}")
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