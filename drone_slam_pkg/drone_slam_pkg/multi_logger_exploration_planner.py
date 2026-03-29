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
    map_meta.csv          — (t, source, width, height, resolution_m,
                               free_cells, occupied_cells, unknown_cells,
                               entropy, update_seq)
    map_compare.csv       — (t, drone_ns, overlap_pct, disagreement_pct,
                               drone_extra_explored_pct, merged_extra_explored_pct)
    maps/
      <source>_<t>s.png   — colour-coded occupancy snapshots
    summary.json          — aggregated stats written on shutdown
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32, Int32, Bool
from nav_msgs.msg import OccupancyGrid, Path
from px4_msgs.msg import VehicleLocalPosition
import statistics
import json
import os
import csv
import math
import numpy as np
from datetime import datetime

# Optional PNG writing — falls back to PGM if Pillow is unavailable
try:
    from PIL import Image as PILImage
    _HAS_PIL = True
except ImportError:
    _HAS_PIL = False


# ──────────────────────────────────────────────────────────────────────────────
#  Helpers
# ──────────────────────────────────────────────────────────────────────────────

def _occ_to_rgba(grid_data: np.ndarray) -> np.ndarray:
    """
    Convert a flat OccupancyGrid data array to an RGBA image array.
      -1  → unknown  → (128, 128, 128, 255)  mid-gray
       0  → free     → (255, 255, 255, 255)  white
     100  → occupied → (30,  30,  30,  255)  near-black
    Values 1-99 are interpolated from white→black.
    """
    h, w = grid_data.shape
    rgba = np.zeros((h, w, 4), dtype=np.uint8)
    rgba[..., 3] = 255  # fully opaque

    unknown_mask   = grid_data == -1
    free_mask      = grid_data == 0
    occupied_mask  = grid_data == 100
    graded_mask    = (~unknown_mask) & (~free_mask) & (~occupied_mask)

    rgba[unknown_mask]  = [128, 128, 128, 255]
    rgba[free_mask]     = [255, 255, 255, 255]
    rgba[occupied_mask] = [30,  30,  30,  255]

    if graded_mask.any():
        vals = grid_data[graded_mask].astype(np.float32) / 100.0   # 0→1
        intensity = (255 * (1.0 - vals)).astype(np.uint8)
        rgba[graded_mask, 0] = intensity
        rgba[graded_mask, 1] = intensity
        rgba[graded_mask, 2] = intensity

    return rgba


def _compute_entropy(grid_data: np.ndarray) -> float:
    """
    Mean per-cell entropy (bits).
    Unknown cells (-1) → max entropy = 1.0 bit.
    Known cells      → -p*log2(p) - (1-p)*log2(1-p)  with p = value/100.
    """
    flat = grid_data.flatten().astype(np.float32)
    total = len(flat)
    if total == 0:
        return 0.0

    unknown_count = np.sum(flat == -1)
    known = flat[(flat >= 0) & (flat <= 100)] / 100.0

    eps = 1e-9
    p = np.clip(known, eps, 1 - eps)
    cell_entropy = -(p * np.log2(p) + (1 - p) * np.log2(1 - p))

    total_entropy = float(np.sum(cell_entropy)) + float(unknown_count) * 1.0
    return round(total_entropy / total, 6)


def _cell_counts(grid_data: np.ndarray):
    flat = grid_data.flatten()
    free     = int(np.sum(flat == 0))
    occupied = int(np.sum(flat == 100))
    unknown  = int(np.sum(flat == -1))
    return free, occupied, unknown


def _save_map_png(rgba: np.ndarray, path: str):
    """Save RGBA array as PNG (Pillow) or raw PGM fallback."""
    if _HAS_PIL:
        PILImage.fromarray(rgba, "RGBA").save(path)
    else:
        pgm_path = path.replace(".png", ".pgm")
        gray = rgba[..., 0]                       # R channel ≡ intensity here
        h, w = gray.shape
        with open(pgm_path, "wb") as f:
            f.write(f"P5\n{w} {h}\n255\n".encode())
            f.write(gray.tobytes())


class MapSnapshot:
    """Holds the latest OccupancyGrid for one source and the parsed numpy array."""
    def __init__(self):
        self.msg:        OccupancyGrid | None = None
        self.data:       np.ndarray   | None = None   # 2-D (height × width)
        self.update_seq: int                 = 0

    def ingest(self, msg: OccupancyGrid):
        self.msg        = msg
        self.update_seq += 1
        w = msg.info.width
        h = msg.info.height
        self.data = np.array(msg.data, dtype=np.int16).reshape((h, w))


# ──────────────────────────────────────────────────────────────────────────────
#  Main node
# ──────────────────────────────────────────────────────────────────────────────

class ExplorationLogger(Node):
    def __init__(self):
        super().__init__("exploration_logger")

        # ── parameters ────────────────────────────────────────────────────────
        self.declare_parameter("log_dir",               "logs")
        self.declare_parameter("drone_namespace",        "")
        self.declare_parameter("drone_namespaces",       ["x500_drone_0", "x500_drone_1"])
        self.declare_parameter("merged_map_topic",       "/merged_map")
        self.declare_parameter("individual_map_suffix",  "/map")
        self.declare_parameter("map_snapshot_interval_s", 10.0)

        log_base        = self.get_parameter("log_dir").value
        self.ns         = self.get_parameter("drone_namespace").value
        self.drone_nss  = self.get_parameter("drone_namespaces").value
        merged_topic    = self.get_parameter("merged_map_topic").value
        map_suffix      = self.get_parameter("individual_map_suffix").value
        snap_interval   = self.get_parameter("map_snapshot_interval_s").value

        # ── run directory ──────────────────────────────────────────────────────
        timestamp     = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_name      = f"run_{timestamp}" if not self.ns else f"run_{self.ns}_{timestamp}"
        self.run_dir  = os.path.join(log_base, run_name)
        self.maps_dir = os.path.join(self.run_dir, "maps")
        os.makedirs(self.maps_dir, exist_ok=True)
        self.get_logger().info(f"Logging to: {self.run_dir}")

        # ── CSV / file handles ─────────────────────────────────────────────────
        self._files   = {}
        self._writers = {}

        self._open_csv("coverage",    ["t", "coverage_pct", "explored_area_m2",
                                       "coverage_rate_m2s", "path_efficiency_m2pm"])
        self._open_csv("path",        ["t", "path_length_m", "avg_speed_ms"])
        self._open_csv("planning",    ["t", "rrt_solve_ms", "rrt_iterations",
                                       "rrt_failed", "frontier_count"])
        self._open_csv("safety",      ["t", "clearance_m", "min_clearance_running_m",
                                        "apf_force_ratio", "tracking_error_m"])
        self._open_csv("cbf",         ["t", "h1", "h2", "phi1_m", "phi2_m",
                                        "slack", "cbf1_active", "cbf2_active",
                                        "speed_clipped", "soft_delta"])
        self._open_csv("events",      ["t", "event_type"])
        self._open_csv("apf_internal",["t", "local_min_count"])
        self._open_csv("multi_drone", ["t", "other_drone_dist_m",
                                        "repulsion_mag", "repulsion_active",
                                        "in_safety_radius"])
        # NEW ──────────────────────────────────────────────────────────────────
        self._open_csv("map_meta",    ["t", "source", "width", "height",
                                       "resolution_m", "free_cells",
                                       "occupied_cells", "unknown_cells",
                                       "entropy", "update_seq"])
        self._open_csv("map_compare", ["t", "drone_ns",
                                       "overlap_pct",
                                       "disagreement_pct",
                                       "drone_extra_explored_pct",
                                       "merged_extra_explored_pct"])
        # ──────────────────────────────────────────────────────────────────────

        self._start_time = self.get_clock().now()

        # ── metrics state ──────────────────────────────────────────────────────
        self._last_coverage_pct   = 0.0
        self._last_area           = 0.0
        self._last_rate           = 0.0
        self._last_efficiency     = 0.0
        self._coverage_samples    = []
        self._last_path_length    = 0.0
        self._last_avg_speed      = 0.0
        self._rrt_times           = []
        self._rrt_iters           = []
        self._rrt_fail_count      = 0
        self._last_frontier_count = 0
        self._last_clearance      = float('inf')
        self._global_min_clear    = float('inf')
        self._last_force_ratio    = 0.0
        self._force_ratio_samples = []
        self._last_tracking_error = 0.0
        self._tracking_error_samples = []
        self._last_h1             = 0.0
        self._last_h2             = 0.0
        self._last_phi1           = 0.0
        self._last_phi2           = 0.0
        self._last_cbf_slack      = 0.0
        self._last_cbf1_active    = False
        self._last_cbf2_active    = False
        self._last_speed_clipped  = False
        self._last_soft_delta     = 0.0
        self._cbf_infeasible_count = 0
        self._cbf_h1_samples      = []
        self._cbf_h2_samples      = []
        self._cbf_slack_samples   = []
        self._cbf_phi1_samples    = []
        self._cbf_phi2_samples    = []
        self._cbf1_active_count   = 0
        self._cbf2_active_count   = 0
        self._cbf_speed_clip_count = 0
        self._cbf_total_ticks     = 0
        self._stuck_count         = 0
        self._escape_count        = 0
        self._replan_count        = 0
        self._last_drone_dist     = float('inf')
        self._last_drone_rep_mag  = 0.0
        self._last_drone_active   = False
        self._last_drone_in_radius = False
        self._drone_dist_samples  = []
        self._drone_in_radius_ticks = 0
        self._drone_total_ticks   = 0
        self._actual_trajectory   = []
        self._rrt_paths           = []

        # ── MAP state ──────────────────────────────────────────────────────────
        # key: source label (e.g. "drone_0", "drone_1", "merged")
        self._map_snapshots: dict[str, MapSnapshot] = {}

        # Pre-create snapshot holders for every tracked source
        for dns in self.drone_nss:
            self._map_snapshots[dns] = MapSnapshot()
        self._map_snapshots["merged"] = MapSnapshot()

        # Entropy history per source for summary
        self._entropy_history: dict[str, list] = {k: [] for k in self._map_snapshots}

        # ── QoS profiles ──────────────────────────────────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_map = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        def topic(name):
            return f"/{self.ns}/{name}" if self.ns else f"/{name}"

        # ── existing metric subscriptions ─────────────────────────────────────
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
        self.create_subscription(Float32, topic("metrics/path_length_m"),
                                 self._cb_path_length,    qos)
        self.create_subscription(Float32, topic("metrics/avg_speed"),
                                 self._cb_avg_speed,      qos)
        self.create_subscription(Float32, topic("metrics/rrt_solve_time_ms"),
                                 self._cb_rrt_time,       qos)
        self.create_subscription(Int32,   topic("metrics/rrt_iterations_used"),
                                 self._cb_rrt_iters,      qos)
        self.create_subscription(Bool,    topic("metrics/rrt_failed"),
                                 self._cb_rrt_fail,       qos)
        self.create_subscription(Int32,   topic("metrics/frontier_count"),
                                 self._cb_frontier_count, qos)
        self.create_subscription(Float32, topic("metrics/obstacle_clearance_m"),
                                 self._cb_clearance,      qos)
        self.create_subscription(Float32, topic("metrics/min_clearance_running"),
                                 self._cb_min_clearance,  qos)
        self.create_subscription(Float32, topic("metrics/apf_force_ratio"),
                                 self._cb_force_ratio,    qos)
        self.create_subscription(Float32, topic("metrics/cbf_h1"),
                                 self._cb_cbf_h1,         qos)
        self.create_subscription(Float32, topic("metrics/cbf_h2"),
                                 self._cb_cbf_h2,         qos)
        self.create_subscription(Float32, topic("metrics/cbf_phi1_m"),
                                 self._cb_cbf_phi1,       qos)
        self.create_subscription(Float32, topic("metrics/cbf_phi2_m"),
                                 self._cb_cbf_phi2,       qos)
        self.create_subscription(Float32, topic("metrics/cbf_slack"),
                                 self._cb_cbf_slack,      qos)
        self.create_subscription(Bool,    topic("metrics/cbf1_obstacle_active"),
                                 self._cb_cbf1_active,    qos)
        self.create_subscription(Bool,    topic("metrics/cbf2_frontier_active"),
                                 self._cb_cbf2_active,    qos)
        self.create_subscription(Int32,   topic("metrics/cbf_infeasible_count"),
                                 self._cb_cbf_infeasible, qos)
        self.create_subscription(Float32, topic("metrics/cbf_soft_delta"),
                                 self._cb_cbf_delta,      qos)
        self.create_subscription(Bool,    topic("metrics/cbf_speed_clipped"),
                                 self._cb_cbf_speed_clipped, qos)
        self.create_subscription(Bool,    topic("metrics/stuck_event"),
                                 self._cb_stuck,          qos)
        self.create_subscription(Bool,    topic("metrics/escape_triggered"),
                                 self._cb_escape,         qos)
        self.create_subscription(Bool,    topic("metrics/rrt_replan_triggered"),
                                 self._cb_replan,         qos)
        self.create_subscription(Float32, topic("metrics/other_drone_distance_m"),
                                 self._cb_drone_dist,        qos)
        self.create_subscription(Float32, topic("metrics/other_drone_repulsion_mag"),
                                 self._cb_drone_rep_mag,     qos)
        self.create_subscription(Bool,    topic("metrics/other_drone_repulsion_active"),
                                 self._cb_drone_rep_active,  qos)
        self.create_subscription(Bool,    topic("metrics/other_drone_in_safety_radius"),
                                 self._cb_drone_in_radius,   qos)
        self.create_subscription(Int32,   topic("metrics/local_min_count"),
                                 self._cb_local_min,      qos)
        self.create_subscription(VehicleLocalPosition,
            "fmu/out/vehicle_local_position", self._cb_pos, qos_px4)
        self.create_subscription(Path,
            "global_path", self._cb_rrt_path, qos_reliable)

        # ── MAP subscriptions ──────────────────────────────────────────────────
        # Individual drone maps
        for dns in self.drone_nss:
            _dns_capture = dns   # capture for lambda closure
            t = f"/{dns}{map_suffix}"
            self.create_subscription(
                OccupancyGrid, t,
                lambda msg, label=_dns_capture: self._cb_map(msg, label),
                qos_map
            )
            self.get_logger().info(f"  Subscribed to individual map: {t}")

        # Merged map
        self.create_subscription(
            OccupancyGrid, merged_topic,
            lambda msg: self._cb_map(msg, "merged"),
            qos_map
        )
        self.get_logger().info(f"  Subscribed to merged map: {merged_topic}")

        # ── row buffers & timers ───────────────────────────────────────────────
        self._row_buffer = {k: [] for k in
                            ["coverage", "path", "planning", "safety", "cbf",
                             "events", "apf_internal", "multi_drone",
                             "map_meta", "map_compare"]}

        self.create_timer(1.0,            self._write_coverage_row)
        self.create_timer(0.1,            self._write_safety_row)
        self.create_timer(0.1,            self._write_cbf_row)
        self.create_timer(1.0,            self._flush_buffers)
        self.create_timer(0.5,            self._write_multi_drone_row)
        self.create_timer(snap_interval,  self._map_snapshot_tick)   # NEW

        self.get_logger().info("Exploration Logger ONLINE")
        self.get_logger().info(f"  Namespace   : '{self.ns}' (empty = single drone)")
        self.get_logger().info(f"  Drone NSs   : {self.drone_nss}")
        self.get_logger().info(f"  Map interval: {snap_interval} s")
        self.get_logger().info(f"  PIL present : {_HAS_PIL} "
                                f"({'PNG' if _HAS_PIL else 'PGM fallback'})")
        self.get_logger().info(f"  Run dir     : {self.run_dir}")

    # ──────────────────────────────────────────────────────────────────────────
    #  CSV helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _open_csv(self, name: str, headers: list):
        path = os.path.join(self.run_dir, f"{name}.csv")
        f = open(path, "w", newline="")
        w = csv.DictWriter(f, fieldnames=headers)
        w.writeheader()
        self._files[name]   = f
        self._writers[name] = w

    def _buf(self, sheet: str, row: dict):
        self._row_buffer[sheet].append(row)

    def _flush_buffers(self):
        for sheet, rows in self._row_buffer.items():
            if rows:
                for row in rows:
                    self._writers[sheet].writerow(row)
                self._files[sheet].flush()
                self._row_buffer[sheet] = []

    # ──────────────────────────────────────────────────────────────────────────
    #  Map callbacks & snapshot logic
    # ──────────────────────────────────────────────────────────────────────────

    def _cb_map(self, msg: OccupancyGrid, label: str):
        """Store latest map for each source; metadata logged on each update."""
        snap = self._map_snapshots.get(label)
        if snap is None:
            # Dynamically encountered drone not in the parameter list
            self._map_snapshots[label] = MapSnapshot()
            self._entropy_history[label] = []
            snap = self._map_snapshots[label]

        snap.ingest(msg)
        t = self._t()

        free, occ, unk = _cell_counts(snap.data)
        ent = _compute_entropy(snap.data)
        self._entropy_history[label].append(ent)

        self._buf("map_meta", {
            "t":            round(t, 3),
            "source":       label,
            "width":        msg.info.width,
            "height":       msg.info.height,
            "resolution_m": round(msg.info.resolution, 5),
            "free_cells":   free,
            "occupied_cells": occ,
            "unknown_cells":  unk,
            "entropy":      ent,
            "update_seq":   snap.update_seq,
        })

    def _map_snapshot_tick(self):
        """
        Periodic timer — save PNG for every available map and compute
        disagreement between each individual drone map and the merged map.
        """
        t = round(self._t(), 1)
        merged_snap = self._map_snapshots.get("merged")

        for label, snap in self._map_snapshots.items():
            if snap.data is None:
                continue

            # ── save PNG ──────────────────────────────────────────────────────
            fname = f"{label}_{t}s.png"
            fpath = os.path.join(self.maps_dir, fname)
            rgba  = _occ_to_rgba(snap.data)
            _save_map_png(rgba, fpath)

            # ── compute disagreement vs merged ────────────────────────────────
            if label == "merged" or merged_snap is None or merged_snap.data is None:
                continue

            self._compute_and_log_comparison(t, label, snap, merged_snap)

        self.get_logger().debug(f"Map snapshot at t={t}s written to {self.maps_dir}")

    def _compute_and_log_comparison(
        self,
        t: float,
        drone_ns: str,
        drone_snap: MapSnapshot,
        merged_snap: MapSnapshot,
    ):
        """
        Align drone map and merged map to the same frame-of-reference origin
        and compute cell-level agreement statistics.

        If the maps have different sizes (common during exploration), we crop
        to the overlapping region using their respective origins.  Cells are
        classified as:
          - free     : value == 0
          - occupied : value == 100
          - known    : free or occupied

        Metrics:
          overlap_pct              — fraction of cells where both maps are known
          disagreement_pct         — among overlapping known cells, fraction where
                                     free↔occupied (real conflict)
          drone_extra_explored_pct — cells known in drone map but unknown in merged
          merged_extra_explored_pct— cells known in merged map but unknown in drone
        """
        if drone_snap.msg is None or merged_snap.msg is None:
            return

        # Resolution must match for cell-level comparison to be meaningful
        res_d = drone_snap.msg.info.resolution
        res_m = merged_snap.msg.info.resolution
        if abs(res_d - res_m) > 1e-4:
            self.get_logger().warn(
                f"Map comparison skipped for {drone_ns}: "
                f"resolution mismatch ({res_d:.4f} vs {res_m:.4f})"
            )
            return

        # Origins in world coords
        ox_d = drone_snap.msg.info.origin.position.x
        oy_d = drone_snap.msg.info.origin.position.y
        ox_m = merged_snap.msg.info.origin.position.x
        oy_m = merged_snap.msg.info.origin.position.y

        # Offset of drone origin relative to merged origin in cells
        off_col = round((ox_d - ox_m) / res_d)
        off_row = round((oy_d - oy_m) / res_d)

        dh, dw = drone_snap.data.shape
        mh, mw = merged_snap.data.shape

        # Intersection in merged-map cell indices
        m_col_start = max(0, off_col)
        m_row_start = max(0, off_row)
        m_col_end   = min(mw, off_col + dw)
        m_row_end   = min(mh, off_row + dh)

        if m_col_end <= m_col_start or m_row_end <= m_row_start:
            # No overlap yet — skip logging
            return

        # Corresponding slice in drone map
        d_col_start = m_col_start - off_col
        d_row_start = m_row_start - off_row
        d_col_end   = m_col_end   - off_col
        d_row_end   = m_row_end   - off_row

        d_region = drone_snap.data [d_row_start:d_row_end, d_col_start:d_col_end]
        m_region = merged_snap.data[m_row_start:m_row_end, m_col_start:m_col_end]

        total_cells = d_region.size
        if total_cells == 0:
            return

        d_known = (d_region >= 0)   # True where drone has a free/occ reading
        m_known = (m_region >= 0)

        both_known = d_known & m_known
        overlap_count = int(np.sum(both_known))

        # Disagreement: one says free (0), other says occupied (100)
        if overlap_count > 0:
            d_free_m_occ = (d_region == 0)   & (m_region == 100)
            d_occ_m_free = (d_region == 100) & (m_region == 0)
            disagree_count   = int(np.sum((d_free_m_occ | d_occ_m_free) & both_known))
            disagreement_pct = round(100.0 * disagree_count / overlap_count, 4)
        else:
            disagreement_pct = None

        overlap_pct             = round(100.0 * overlap_count / total_cells, 4)
        drone_extra_explored    = int(np.sum(d_known & ~m_known))
        merged_extra_explored   = int(np.sum(~d_known & m_known))
        drone_extra_pct         = round(100.0 * drone_extra_explored  / total_cells, 4)
        merged_extra_pct        = round(100.0 * merged_extra_explored / total_cells, 4)

        self._buf("map_compare", {
            "t":                        t,
            "drone_ns":                 drone_ns,
            "overlap_pct":              overlap_pct,
            "disagreement_pct":         disagreement_pct if disagreement_pct is not None else "",
            "drone_extra_explored_pct": drone_extra_pct,
            "merged_extra_explored_pct": merged_extra_pct,
        })

    # ──────────────────────────────────────────────────────────────────────────
    #  Existing callbacks — unchanged
    # ──────────────────────────────────────────────────────────────────────────

    def _cb_pos(self, msg):
        if (len(self._actual_trajectory) == 0 or
                (round(self._t(), 1) - self._actual_trajectory[-1][0]) >= 0.2):
            self._actual_trajectory.append((round(self._t(), 3), msg.y, msg.x))

    def _cb_rrt_path(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self._rrt_paths.append({"t": round(self._t(), 3), "path": pts})

    def _t(self) -> float:
        return (self.get_clock().now() - self._start_time).nanoseconds / 1e9

    def _cb_tracking_error(self, msg: Float32):
        self._last_tracking_error = msg.data
        self._tracking_error_samples.append(msg.data)

    def _cb_coverage_pct(self, msg: Float32):
        self._last_coverage_pct = msg.data

    def _cb_area(self, msg: Float32):
        self._last_area = msg.data

    def _cb_coverage_rate(self, msg: Float32):
        self._last_rate = msg.data

    def _cb_efficiency(self, msg: Float32):
        self._last_efficiency = msg.data

    def _cb_drone_dist(self, msg: Float32):
        self._last_drone_dist = msg.data
        self._drone_dist_samples.append(msg.data)

    def _cb_drone_rep_mag(self, msg: Float32):
        self._last_drone_rep_mag = msg.data

    def _cb_drone_rep_active(self, msg: Bool):
        self._last_drone_active = msg.data
        if msg.data:
            self._buf("events", {"t": round(self._t(), 3),
                                  "event_type": "drone_repulsion_active"})

    def _cb_drone_in_radius(self, msg: Bool):
        self._last_drone_in_radius = msg.data
        self._drone_total_ticks += 1
        if msg.data:
            self._drone_in_radius_ticks += 1

    def _write_multi_drone_row(self):
        self._buf("multi_drone", {
            "t":                  round(self._t(), 3),
            "other_drone_dist_m": round(self._last_drone_dist, 4)
                                   if self._last_drone_dist != float('inf') else "",
            "repulsion_mag":      round(self._last_drone_rep_mag, 4),
            "repulsion_active":   int(self._last_drone_active),
            "in_safety_radius":   int(self._last_drone_in_radius),
        })

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
            "t":             round(self._t(), 3),
            "path_length_m": round(msg.data, 4),
            "avg_speed_ms":  round(self._last_avg_speed, 4),
        })

    def _cb_avg_speed(self, msg: Float32):
        self._last_avg_speed = msg.data

    def _cb_rrt_time(self, msg: Float32):
        self._rrt_times.append(msg.data)
        self._buf("planning", {
            "t":              round(self._t(), 3),
            "rrt_solve_ms":   round(msg.data, 2),
            "rrt_iterations": self._rrt_iters[-1] if self._rrt_iters else "",
            "rrt_failed":     0,
            "frontier_count": self._last_frontier_count,
        })

    def _cb_rrt_iters(self, msg: Int32):
        self._rrt_iters.append(msg.data)

    def _cb_rrt_fail(self, msg: Bool):
        if msg.data:
            self._rrt_fail_count += 1
            self._buf("planning", {
                "t":              round(self._t(), 3),
                "rrt_solve_ms":   "",
                "rrt_iterations": "",
                "rrt_failed":     1,
                "frontier_count": self._last_frontier_count,
            })

    def _cb_frontier_count(self, msg: Int32):
        self._last_frontier_count = msg.data
        self._buf("planning", {
            "t":              round(self._t(), 3),
            "rrt_solve_ms":   "",
            "rrt_iterations": "",
            "rrt_failed":     0,
            "frontier_count": msg.data,
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
        self._buf("cbf", {
            "t":             round(self._t(), 3),
            "h1":            round(self._last_h1,  4),
            "h2":            round(self._last_h2,  4),
            "phi1_m":        round(self._last_phi1, 4),
            "phi2_m":        round(self._last_phi2, 4),
            "slack":         round(self._last_cbf_slack, 5),
            "cbf1_active":   int(self._last_cbf1_active),
            "cbf2_active":   int(self._last_cbf2_active),
            "speed_clipped": int(self._last_speed_clipped),
            "soft_delta":    round(self._last_soft_delta, 6),
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

    # ──────────────────────────────────────────────────────────────────────────
    #  Summary
    # ──────────────────────────────────────────────────────────────────────────

    def _write_summary(self):
        total_time = self._t()

        def time_to_pct(target):
            for t, pct in self._coverage_samples:
                if pct >= target:
                    return round(t, 2)
            return None

        avg_rrt_ms  = round(sum(self._rrt_times) / len(self._rrt_times), 2) \
                      if self._rrt_times else None
        med_rrt_ms  = round(statistics.median(self._rrt_times), 2) \
                      if self._rrt_times else None
        avg_iters   = round(sum(self._rrt_iters) / len(self._rrt_iters), 1) \
                      if self._rrt_iters else None
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

        def safe_min(lst):
            return round(min(lst), 4) if lst else None

        cbf_intervention_rate = round(
            (self._cbf1_active_count + self._cbf2_active_count) /
            max(self._cbf_total_ticks * 2, 1), 4
        ) if self._cbf_total_ticks else None

        # ── map summary ───────────────────────────────────────────────────────
        map_summary = {}
        for label, snap in self._map_snapshots.items():
            if snap.data is None:
                map_summary[label] = {"status": "no_data"}
                continue
            free, occ, unk = _cell_counts(snap.data)
            total = snap.data.size
            eh = self._entropy_history.get(label, [])
            map_summary[label] = {
                "final_width":          snap.msg.info.width,
                "final_height":         snap.msg.info.height,
                "resolution_m":         round(snap.msg.info.resolution, 5),
                "total_cells":          total,
                "final_free_cells":     free,
                "final_occupied_cells": occ,
                "final_unknown_cells":  unk,
                "final_known_pct":      round(100.0 * (free + occ) / max(total, 1), 3),
                "final_entropy":        round(eh[-1], 6) if eh else None,
                "min_entropy_seen":     round(min(eh), 6) if eh else None,
                "avg_entropy":          round(sum(eh) / len(eh), 6) if eh else None,
                "total_updates":        snap.update_seq,
            }

        # Final snapshot of all maps on shutdown
        for label, snap in self._map_snapshots.items():
            if snap.data is not None:
                fname = f"{label}_FINAL.png"
                rgba  = _occ_to_rgba(snap.data)
                _save_map_png(rgba, os.path.join(self.maps_dir, fname))

        with open(os.path.join(self.run_dir, "trajectory.json"), "w") as f:
            json.dump({"actual": self._actual_trajectory,
                       "rrt_paths": self._rrt_paths}, f)

        summary = {
            "run_dir":                  self.run_dir,
            "drone_namespace":          self.ns,
            "total_exploration_time_s": round(total_time, 2),
            "final_coverage_pct":       round(self._last_coverage_pct, 2),
            "final_explored_area_m2":   round(self._last_area, 3),
            "time_to_25pct_s":          time_to_pct(25.0),
            "time_to_50pct_s":          time_to_pct(50.0),
            "time_to_75pct_s":          time_to_pct(75.0),
            "time_to_90pct_s":          time_to_pct(90.0),
            "total_path_length_m":      round(self._last_path_length, 3),
            "avg_speed_ms":             round(self._last_avg_speed, 4),
            "final_path_efficiency":    round(self._last_efficiency, 4),
            "total_replans":            self._replan_count,
            "total_rrt_failures":       self._rrt_fail_count,
            "avg_rrt_solve_ms":         avg_rrt_ms,
            "median_rrt_solve_ms":      med_rrt_ms,
            "avg_rrt_iterations":       avg_iters,
            "rrt_success_rate_pct":     round(
                100.0 * len(self._rrt_times) /
                max(len(self._rrt_times) + self._rrt_fail_count, 1), 2),
            "global_min_clearance_m":   round(self._global_min_clear, 4)
                                        if self._global_min_clear != float('inf') else None,
            "avg_apf_force_ratio":      avg_ratio,
            "max_apf_force_ratio":      max_ratio,
            "avg_tracking_error_m":     avg_tracking,
            "max_tracking_error_m":     max_tracking,
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
            "drone_min_separation_m":      round(min(self._drone_dist_samples), 4)
                                           if self._drone_dist_samples else None,
            "drone_avg_separation_m":      safe_avg(self._drone_dist_samples),
            "drone_in_radius_ticks":       self._drone_in_radius_ticks,
            "drone_total_ticks":           self._drone_total_ticks,
            "drone_proximity_rate":        round(
                self._drone_in_radius_ticks / max(self._drone_total_ticks, 1), 4
            ) if self._drone_total_ticks else None,
            "total_stuck_events":          self._stuck_count,
            "total_escape_events":         self._escape_count,
            # NEW
            "maps":                        map_summary,
        }

        path = os.path.join(self.run_dir, "summary.json")
        with open(path, "w") as f:
            json.dump(summary, f, indent=2)

        # ── console summary ───────────────────────────────────────────────────
        self.get_logger().info("=" * 55)
        self.get_logger().info("LOGGER SUMMARY")
        self.get_logger().info(f"  Total time       : {round(total_time, 1)} s")
        self.get_logger().info(f"  Final coverage   : {round(self._last_coverage_pct, 1)} %")
        self.get_logger().info(f"  Path length      : {round(self._last_path_length, 2)} m")
        self.get_logger().info(f"  RRT* avg solve   : {avg_rrt_ms} ms")
        self.get_logger().info(f"  Stuck events     : {self._stuck_count}")
        min_str = (f"{round(self._global_min_clear, 3)} m"
                   if self._global_min_clear != float('inf') else "N/A")
        self.get_logger().info(f"  Min clearance    : {min_str}")
        self.get_logger().info(f"  CBF ticks        : {self._cbf_total_ticks}")
        self.get_logger().info(f"  CBF c1 active    : {self._cbf1_active_count} ticks")
        self.get_logger().info(f"  CBF c2 active    : {self._cbf2_active_count} ticks")
        self.get_logger().info(f"  CBF intervention : {cbf_intervention_rate}")
        self.get_logger().info(f"  CBF infeasible   : {self._cbf_infeasible_count}")
        self.get_logger().info(f"  CBF speed clips  : {self._cbf_speed_clip_count}")
        min_sep  = (round(min(self._drone_dist_samples), 3)
                    if self._drone_dist_samples else "N/A")
        avg_sep  = safe_avg(self._drone_dist_samples) or "N/A"
        prox_pct = round(100.0 * self._drone_in_radius_ticks /
                         max(self._drone_total_ticks, 1), 1) \
                   if self._drone_total_ticks else "N/A"
        self.get_logger().info(f"  Drone min sep    : {min_sep} m")
        self.get_logger().info(f"  Drone avg sep    : {avg_sep} m")
        self.get_logger().info(f"  In safety radius : {prox_pct} % of ticks")
        track_str = (f"{avg_tracking} m (max: {max_tracking} m)"
                     if avg_tracking else "N/A")
        self.get_logger().info(f"  Tracking error   : {track_str}")
        self.get_logger().info("  ── Maps ────────────────────────────────────")
        for label, ms in map_summary.items():
            if ms.get("status") == "no_data":
                self.get_logger().info(f"    [{label}] no data received")
            else:
                self.get_logger().info(
                    f"[{label}] known={ms['final_known_pct']}%",
                    f"entropy={ms['final_entropy']}  ",
                    f"updates={ms['total_updates']}",
                )
        self.get_logger().info(f"  Summary written  : {path}")
        self.get_logger().info("=" * 55)

    # ──────────────────────────────────────────────────────────────────────────

    def destroy_node(self):
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
