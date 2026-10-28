import copy
import math
import time
from array import array

import cv2
import numpy as np
import rclpy
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool

from pomona_live_costmap_real import rotation_matrix
from pomona_live_costmap_real_v2 import RealCostmapV2, select_scan, stamp_key


def apply_finite_rays(live, scan, tf, info):
    """Same resolution/3 ray sampling, grouped to reduce Python overhead."""
    resolution = info.resolution
    numbers = (scan.range_min, scan.range_max, scan.angle_min, scan.angle_increment, resolution)
    if (not all(math.isfinite(v) for v in numbers) or resolution <= 0
            or scan.range_min < 0 or scan.range_max <= scan.range_min):
        raise ValueError('Invalid scan/map metadata')
    ranges = np.asarray(scan.ranges, dtype=float)
    indices = np.flatnonzero(np.isfinite(ranges) & (ranges >= scan.range_min)
                             & (ranges < scan.range_max))
    if not len(indices):
        return 0
    distances = ranges[indices]
    angles = scan.angle_min + indices * scan.angle_increment
    vectors = np.column_stack((np.cos(angles), np.sin(angles), np.zeros(len(angles))))
    origin = info.origin
    offset = np.array([origin.position.x, origin.position.y, origin.position.z])
    t = tf.translation
    translation = np.array([t.x, t.y, t.z])
    q_values = [tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w,
                origin.orientation.x, origin.orientation.y, origin.orientation.z, origin.orientation.w]
    if not np.all(np.isfinite(np.r_[offset, translation, q_values])):
        raise ValueError('Nonfinite transform/map origin')
    map_rotation = rotation_matrix(origin.orientation)
    directions = (vectors @ rotation_matrix(tf.rotation).T) @ map_rotation
    start = (translation-offset) @ map_rotation
    if not np.all(np.isfinite(directions)):
        raise ValueError('Invalid ray directions')
    step = resolution/3.0
    clear_ends = np.maximum(scan.range_min, distances-step)
    height, width = live.shape
    # Bounded temporary allocation, even when scan.range_max is very large.
    for begin in range(0, len(indices), 16):
        ends = clear_ends[begin:begin+16]
        samples = np.arange(scan.range_min, float(np.max(ends)), step)
        if not samples.size:
            continue
        for sample_begin in range(0, len(samples), 2048):
            sample_chunk = samples[sample_begin:sample_begin+2048]
            observed = sample_chunk[None, :] < ends[:, None]
            points = (start[None,None,:2] + sample_chunk[None,:,None]
                      * directions[begin:begin+16,None,:2])
            ij = np.floor(points/resolution).astype(np.int64)
            valid = observed & (ij[:,:,0] >= 0) & (ij[:,:,0] < width) & (ij[:,:,1] >= 0) & (ij[:,:,1] < height)
            cells = ij[valid]
            live[cells[:,1],cells[:,0]] = False
    # Apply all hits after clearing to preserve endpoints of adjacent rays.
    endpoints = start + distances[:,None]*directions
    cells = np.floor(endpoints[:,:2]/resolution).astype(np.int64)
    valid = ((cells[:,0] >= 0) & (cells[:,0] < width)
             & (cells[:,1] >= 0) & (cells[:,1] < height))
    cells = cells[valid]
    live[cells[:,1],cells[:,0]] = True
    return len(indices)


def inflate_grid(static_grid, live, resolution, radius):
    free = (static_grid == 0) & ~live
    padded = np.pad(free, 1, mode='constant', constant_values=False).astype(np.uint8)
    pixels = cv2.distanceTransform(padded, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
    distance = pixels[1:-1,1:-1].astype(np.float64)*resolution
    blocked = distance <= radius + resolution/math.sqrt(2.0) + 1e-6
    return np.where(blocked, 100, 0).astype(np.int8)


def pack_grid(values):
    data = array('b')
    data.frombytes(np.ascontiguousarray(values, dtype=np.int8).tobytes())
    return data


class RealCostmapV3(RealCostmapV2):
    def __init__(self):
        super().__init__()
        cv2.setNumThreads(1)
        self.static_grid = None
        self.max_encode_ms = self.max_transport_ms = 0.0
        self.last_extra_diag = time.monotonic()
        self.get_logger().info('REAL COSTMAP v3 ACTIVE: batched rays + binary map data; unchanged 0.6s freshness gate')

    def on_map(self, msg):
        info = msg.info
        if (info.width <= 0 or info.height <= 0 or not math.isfinite(info.resolution)
                or info.resolution <= 0 or len(msg.data) != info.width*info.height):
            self.map_msg = None
            self.static_grid = None
            self.last_success_wall = None
            return
        # Parent stores geometry and resets the live layer on geometry changes.
        super().on_map(msg)
        self.static_grid = np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width).copy()

    def work(self):
        if self.map_msg is None or self.static_grid is None:
            return
        self.scans, selected, old, future, waits = select_scan(
            self.scans, self.last_used_key, self.scan_age, self.lookup_scan)
        self.old += old
        self.future += future
        self.tf_wait += waits
        if selected is None:
            return
        scan, tf = selected
        self.last_used_key = stamp_key(scan)
        if not -0.05 <= self.scan_age(scan) < 0.5:
            self.old += 1
            return
        before_live = self.live.copy()
        start = time.monotonic()
        try:
            self.finite_beams = apply_finite_rays(self.live, scan, tf, self.map_msg.info)
            self.max_ray_ms = max(self.max_ray_ms, (time.monotonic()-start)*1000)
            if not self.finite_beams:
                self.live = before_live
                self.invalid += 1
                return
            publish_start = time.monotonic()
            values = inflate_grid(self.static_grid, self.live, self.map_msg.info.resolution, self.inflation_radius)
            output = OccupancyGrid()
            output.header = copy.deepcopy(self.map_msg.header)
            output.header.stamp = scan.header.stamp
            output.info = copy.deepcopy(self.map_msg.info)
            output.data = pack_grid(values)
            self.max_encode_ms = max(self.max_encode_ms, (time.monotonic()-publish_start)*1000)
            if not -0.05 <= self.scan_age(scan) < 0.6:
                self.live = before_live
                self.slow += 1
                return
            transport_start = time.monotonic()
            self.publisher.publish(output)
            self.max_transport_ms = max(self.max_transport_ms, (time.monotonic()-transport_start)*1000)
            self.max_publish_ms = max(self.max_publish_ms, (time.monotonic()-publish_start)*1000)
            self.last_success_wall = time.monotonic()
            self.last_success_stamp = Time.from_msg(scan.header.stamp)
            self.ok += 1
            if self.scan_age(scan) >= 0.6:
                self.slow += 1
        except Exception:
            self.live = before_live
            raise

    def update(self):
        super().update()
        wall = time.monotonic()
        if wall-self.last_extra_diag >= 5.0:
            self.get_logger().info(f'V3 encode_max={self.max_encode_ms:.1f}ms publish_call_max={self.max_transport_ms:.1f}ms')
            self.last_extra_diag = wall


def main():
    rclpy.init()
    node = RealCostmapV3()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.ready_pub.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
