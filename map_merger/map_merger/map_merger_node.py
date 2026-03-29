#!/usr/bin/env python3
"""
map_merger_node.py
------------------
Merges two OccupancyGrid maps from two independent RTAB-Map SLAM instances in
real time. The pose of map2's origin expressed in map1's frame must be known
a priori (parameters: map2_x, map2_y, map2_yaw).

Architecture
------------
  /robot1/map  ──┐
                 ├──► MapMergerNode ──► /merged_map
  /robot2/map  ──┘         │
                            └──► static TF: merged_map → robot1/map
                                          merged_map → robot2/map

Each SLAM instance continues its own pose graph independently. The merger
simply reprojects map2's cells into map1's reference frame, unions both grids,
and publishes at a configurable rate.

Merge modes
-----------
  max       : occupied wins (good for conservative obstacle maps)
  average   : probability average of known cells
  overwrite : last map wins (robot2 overrides robot1 in overlap regions)

Alignment / flip parameters (per-map)
--------------------------------------
  map1_flip_x, map2_flip_x  : mirror the map along its local X axis before
                               projecting (fixes left-right reversal)
  map1_flip_y, map2_flip_y  : mirror along local Y (fixes up-down reversal)
  map1_rot90, map2_rot90    : additional 90-degree CCW rotations to apply
                               (0, 1, 2 or 3 steps) before world projection.
                               Useful when RTAB-Map builds the grid in a
                               transposed orientation.

Tuning workflow
---------------
  1. Visualise /robot1/map and /robot2/map in RViz side-by-side.
  2. If a map looks mirrored left-right  → set mapN_flip_x: true
     If a map looks mirrored top-bottom  → set mapN_flip_y: true
     If a map looks rotated 90°          → set mapN_rot90:  1 (or 2 / 3)
  3. Adjust mapN_x / mapN_y / mapN_yaw until the two maps align.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                        QoSHistoryPolicy)
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np



def quat_to_yaw(q) -> float:
    """Extract yaw (rotation about Z) from a geometry_msgs Quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def yaw_to_quat(yaw: float):
    """Return (qz, qw) for a pure Z-rotation."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def apply_grid_transforms(data: np.ndarray, flip_x: bool, flip_y: bool,
                           rot90: int) -> np.ndarray:

    if flip_x:
        data = np.fliplr(data)
    if flip_y:
        data = np.flipud(data)
    if rot90 % 4:
        data = np.rot90(data, k=rot90 % 4)
    return data


class MapMergerNode(Node):

    def __init__(self):
        super().__init__('map_merger')

        self.declare_parameter('map1_topic',      '/robot1/map')
        self.declare_parameter('map2_topic',      '/robot2/map')
        self.declare_parameter('merged_topic',    '/merged_map')
        self.declare_parameter('map1_frame',      'robot1/map')
        self.declare_parameter('map2_frame',      'robot2/map')
        self.declare_parameter('merged_frame',    'merged_map')

        self.declare_parameter('map2_x',   0.0)
        self.declare_parameter('map2_y',   0.0)
        self.declare_parameter('map2_yaw', 0.0)

        self.declare_parameter('map1_x',   0.0)
        self.declare_parameter('map1_y',   0.0)
        self.declare_parameter('map1_yaw', 0.0)

        self.declare_parameter('map1_flip_x', False)   # mirror left↔right
        self.declare_parameter('map1_flip_y', False)   # mirror top↔bottom
        self.declare_parameter('map1_rot90',  0)       # CCW 90° steps (0-3)

        # map2 corrections
        self.declare_parameter('map2_flip_x', False)
        self.declare_parameter('map2_flip_y', False)
        self.declare_parameter('map2_rot90',  0)

        # How often to publish the merged map (Hz)
        self.declare_parameter('publish_rate', 2.0)

        # 'max' | 'average' | 'overwrite'
        self.declare_parameter('merge_mode', 'max')

        # Extra margin in metres added around the merged bounding box
        self.declare_parameter('border_m', 0.5)

        p = self.get_parameter
        self._map1_topic   = p('map1_topic').value
        self._map2_topic   = p('map2_topic').value
        self._merged_topic = p('merged_topic').value
        self._map1_frame   = p('map1_frame').value
        self._map2_frame   = p('map2_frame').value
        self._merged_frame = p('merged_frame').value

        self._rel_x   = p('map2_x').value
        self._rel_y   = p('map2_y').value
        self._rel_yaw = p('map2_yaw').value

        self._m1_x   = p('map1_x').value
        self._m1_y   = p('map1_y').value
        self._m1_yaw = p('map1_yaw').value
        
        self._m1_flip_x = bool(p('map1_flip_x').value)
        self._m1_flip_y = bool(p('map1_flip_y').value)
        self._m1_rot90  = int(p('map1_rot90').value)
        self._m2_flip_x = bool(p('map2_flip_x').value)
        self._m2_flip_y = bool(p('map2_flip_y').value)
        self._m2_rot90  = int(p('map2_rot90').value)

        self._rate      = float(p('publish_rate').value)
        self._mode      = p('merge_mode').value
        self._border_m  = float(p('border_m').value)

        self._map1: OccupancyGrid | None = None
        self._map2: OccupancyGrid | None = None

        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        self.create_subscription(
            OccupancyGrid, self._map1_topic, self._cb_map1, map_qos)
        self.create_subscription(
            OccupancyGrid, self._map2_topic, self._cb_map2, map_qos)

        self._pub = self.create_publisher(
            OccupancyGrid, self._merged_topic, map_qos)

        self._tf_static = tf2_ros.StaticTransformBroadcaster(self)
        self._broadcast_static_tfs()

        self.create_timer(1.0 / self._rate, self._timer_cb)

        self.get_logger().info(
            f'[map_merger] started\n'
            f'  map1 : {self._map1_topic} ({self._map1_frame})\n'
            f'         flip_x={self._m1_flip_x} flip_y={self._m1_flip_y} '
            f'rot90={self._m1_rot90}\n'
            f'  map2 : {self._map2_topic} ({self._map2_frame})\n'
            f'         flip_x={self._m2_flip_x} flip_y={self._m2_flip_y} '
            f'rot90={self._m2_rot90}\n'
            f'  out  : {self._merged_topic} ({self._merged_frame})\n'
            f'  map2 in map1 frame → x={self._rel_x:.3f} m, '
            f'y={self._rel_y:.3f} m, yaw={math.degrees(self._rel_yaw):.1f}°\n'
            f'  merge_mode={self._mode}  rate={self._rate} Hz'
        )


    def _broadcast_static_tfs(self):

        now = self.get_clock().now().to_msg()
        tfs = []

        t1 = TransformStamped()
        t1.header.stamp = now
        t1.header.frame_id = self._merged_frame
        t1.child_frame_id  = self._map1_frame
        t1.transform.rotation.w = 1.0
        tfs.append(t1)

        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = self._merged_frame
        t2.child_frame_id  = self._map2_frame
        t2.transform.translation.x = self._rel_x
        t2.transform.translation.y = self._rel_y
        qz, qw = yaw_to_quat(self._rel_yaw)
        t2.transform.rotation.z = qz
        t2.transform.rotation.w = qw
        tfs.append(t2)

        self._tf_static.sendTransform(tfs)


    def _cb_map1(self, msg: OccupancyGrid):
        self._map1 = msg

    def _cb_map2(self, msg: OccupancyGrid):
        self._map2 = msg


    def _timer_cb(self):
        if self._map1 is None and self._map2 is None:
            return

        if self._map1 is None:
            merged = self._restamp(self._map2)
        elif self._map2 is None:
            merged = self._restamp(self._map1)
        else:
            merged = self._merge(self._map1, self._map2)

        if merged is not None:
            self._pub.publish(merged)

    def _restamp(self, grid: OccupancyGrid) -> OccupancyGrid:
        """Return a shallow copy of grid with the current stamp and merged frame."""
        out = OccupancyGrid()
        out.header.stamp    = self.get_clock().now().to_msg()
        out.header.frame_id = self._merged_frame
        out.info = grid.info
        out.data = grid.data
        return out


    def _merge(self, map1: OccupancyGrid, map2: OccupancyGrid) -> OccupancyGrid | None:

        res = map1.info.resolution

        if abs(map1.info.resolution - map2.info.resolution) > 1e-4:
            self.get_logger().warn(
                f'Resolution mismatch ({map1.info.resolution:.4f} vs '
                f'{map2.info.resolution:.4f}). Using map1 resolution.',
                throttle_duration_sec=10.0,
            )

        def corrected_grid(src: OccupancyGrid, flip_x, flip_y, rot90):
            h, w = src.info.height, src.info.width
            data = np.array(src.data, dtype=np.int16).reshape(h, w)
            data = apply_grid_transforms(data, flip_x, flip_y, rot90)

            new_h, new_w = data.shape
            return data, new_w, new_h

        data1, w1, h1 = corrected_grid(map1,
                                        self._m1_flip_x, self._m1_flip_y,
                                        self._m1_rot90)
        data2, w2, h2 = corrected_grid(map2,
                                        self._m2_flip_x, self._m2_flip_y,
                                        self._m2_rot90)
        m1_oyaw = quat_to_yaw(map1.info.origin.orientation) + self._m1_yaw

        m1_oyaw += math.radians(90 * self._m1_rot90)

        raw_m2_oyaw = quat_to_yaw(map2.info.origin.orientation)
        m2_oyaw = raw_m2_oyaw + self._rel_yaw
        m2_oyaw += math.radians(90 * self._m2_rot90)

        def corners_world(ox, oy, oyaw, w, h):
            c, s = math.cos(oyaw), math.sin(oyaw)
            W, H = w * res, h * res
            pts = np.array([
                [ox,                  oy                ],
                [ox + c * W,          oy + s * W        ],
                [ox + c * W - s * H,  oy + s * W + c * H],
                [ox - s * H,          oy + c * H        ],
            ])
            return pts

        c1 = corners_world(map1.info.origin.position.x,
                        map1.info.origin.position.y,
                        m1_oyaw, w1, h1)

        c2 = corners_world(map2.info.origin.position.x,
                        map2.info.origin.position.y,
                        m2_oyaw, w2, h2)
        all_pts = np.vstack([c1, c2])

        border = self._border_m
        min_x = float(all_pts[:, 0].min()) - border
        min_y = float(all_pts[:, 1].min()) - border
        max_x = float(all_pts[:, 0].max()) + border
        max_y = float(all_pts[:, 1].max()) + border

        mg_w = max(1, int(math.ceil((max_x - min_x) / res)))
        mg_h = max(1, int(math.ceil((max_y - min_y) / res)))
        mg_ox = min_x
        mg_oy = min_y

        merged = np.full(mg_h * mg_w, -1, dtype=np.int16)

        def scatter(data: np.ndarray, src_w: int, src_h: int,
                    ox: float, oy: float, oyaw: float):
            """
            Reproject `data` (already corrected, shape src_h×src_w) into
            the merged flat array using world-frame pose (ox, oy, oyaw).
            """
            cols = np.arange(src_w, dtype=np.float64) + 0.5
            rows = np.arange(src_h, dtype=np.float64) + 0.5
            lx = cols * res   # (src_w,)
            ly = rows * res   # (src_h,)

            c_t, s_t = math.cos(oyaw), math.sin(oyaw)
            lx_g, ly_g = np.meshgrid(lx, ly)   # (src_h, src_w)
            wx = ox + c_t * lx_g - s_t * ly_g
            wy = oy + s_t * lx_g + c_t * ly_g

            mc = np.floor((wx - mg_ox) / res).astype(np.int32)
            mr = np.floor((wy - mg_oy) / res).astype(np.int32)

            in_bounds = (mc >= 0) & (mc < mg_w) & (mr >= 0) & (mr < mg_h)
            known     = in_bounds & (data != -1)

            flat_idx = (mr[known] * mg_w + mc[known]).ravel()
            vals     = data[known].ravel()

            if flat_idx.size == 0:
                return

            dest_vals = merged[flat_idx]
            is_unk    = dest_vals == -1
            merged[flat_idx[is_unk]] = vals[is_unk]

            is_known_dest = ~is_unk
            ki = flat_idx[is_known_dest]
            kv = vals[is_known_dest]

            if ki.size == 0:
                return

            if self._mode == 'max':
                np.maximum.at(merged, ki, kv)
            elif self._mode == 'average':
                avg = ((merged[ki].astype(np.int32) + kv.astype(np.int32))
                       // 2).astype(np.int16)
                merged[ki] = avg
            else:   # overwrite
                merged[ki] = kv

        scatter(data1, w1, h1,
                map1.info.origin.position.x,
                map1.info.origin.position.y,
                m1_oyaw)  

        scatter(data2, w2, h2,
                map2.info.origin.position.x,
                map2.info.origin.position.y,
                m2_oyaw)

        out = OccupancyGrid()
        out.header.stamp    = self.get_clock().now().to_msg()
        out.header.frame_id = self._merged_frame
        out.info.resolution = res
        out.info.width      = mg_w
        out.info.height     = mg_h
        out.info.origin.position.x  = mg_ox
        out.info.origin.position.y  = mg_oy
        out.info.origin.orientation.w = 1.0
        out.data = merged.clip(-1, 100).astype(np.int8).tolist()
        return out




def main(args=None):
    rclpy.init(args=args)
    node = MapMergerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()