#!/usr/bin/env python3
"""Real-robot costmap adapter; keep original simulation files unchanged.

Requires pomona_live_costmap_real.py and pomona_costmap.py in this directory.
Uses a bounded queue and exact scan-time TF without blocking the executor.
Does not relax source-age limits, invent fresh timestamps, or send motion.
No-return/invalid beams do not clear obstacles; finite measured rays do.
"""
import copy
import math
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.time import Time
from rclpy.clock import Clock, ClockType
from std_msgs.msg import Bool
from tf2_ros import TransformException

from pomona_live_costmap_real import LiveCostmap
from pomona_costmap import PomonaCostmap


def stamp_key(scan):
    return (scan.header.stamp.sec, scan.header.stamp.nanosec)


def select_scan(queue, last_key, age_fn, lookup_fn):
    """Newest fresh scan with available exact TF, retaining newer waiters."""
    retained = deque(maxlen=20)
    old = future = 0
    for scan in queue:
        key = stamp_key(scan)
        if last_key is not None and key <= last_key:
            continue
        age = age_fn(scan)
        if not math.isfinite(age) or age >= 0.5:
            old += 1
        elif age < -0.05:
            future += 1
        else:
            retained.append(scan)
    selected = None
    tf_waits = 0
    for scan in reversed(retained):
        try:
            transform = lookup_fn(scan)
        except TransformException:
            tf_waits += 1
            continue
        selected = (scan, transform)
        break
    if selected is not None:
        key = stamp_key(selected[0])
        retained = deque((scan for scan in retained if stamp_key(scan) > key), maxlen=20)
    return retained, selected, old, future, tf_waits


def finite_ranges(scan):
    if (not math.isfinite(scan.range_min) or not math.isfinite(scan.range_max)
            or scan.range_min < 0 or scan.range_max <= scan.range_min
            or not math.isfinite(scan.angle_min) or not math.isfinite(scan.angle_increment)):
        raise ValueError('Invalid scan range/angle metadata')
    return [value if math.isfinite(value) and scan.range_min <= value < scan.range_max
            else math.nan for value in scan.ranges]


class RealCostmapV2(LiveCostmap):
    def __init__(self):
        super().__init__()
        if self.get_parameter('use_sim_time').value:
            raise ValueError('Real costmap requires use_sim_time=false')
        self.scans = deque(maxlen=20)
        self.last_used_key = None
        self.rx = self.ok = self.old = self.future = self.tf_wait = 0
        self.invalid = self.slow = self.overflow = 0
        self.last_rx_wall = None
        self.last_rx_age = math.nan
        self.max_rx_gap = self.max_ray_ms = self.max_publish_ms = 0.0
        self.finite_beams = 0
        self.last_diag = time.monotonic()
        self.destroy_timer(self.timer)
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.05, self.update, clock=self.steady)
        self.get_logger().info('REAL COSTMAP v2: exact-TF scan queue, finite rays only, ready age <0.6s; diagnostics every 5s')

    def scan_age(self, scan):
        return (self.get_clock().now()-Time.from_msg(scan.header.stamp)).nanoseconds*1e-9

    def on_scan(self, msg):
        wall = time.monotonic()
        self.rx += 1
        self.last_rx_age = self.scan_age(msg)
        if self.last_rx_wall is not None:
            self.max_rx_gap = max(self.max_rx_gap, wall-self.last_rx_wall)
        self.last_rx_wall = wall
        if len(self.scans) == self.scans.maxlen:
            self.overflow += 1
        self.scans.append(msg)

    def lookup_scan(self, scan):
        return self.tf_buffer.lookup_transform(
            self.map_msg.header.frame_id, scan.header.frame_id,
            Time.from_msg(scan.header.stamp)).transform

    def work(self):
        if self.map_msg is None:
            return
        if (not math.isfinite(self.map_msg.info.resolution)
                or self.map_msg.info.resolution <= 0):
            raise ValueError('Invalid source map resolution')
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
        filtered = copy.copy(scan)
        filtered.ranges = finite_ranges(scan)
        self.finite_beams = sum(math.isfinite(v) for v in filtered.ranges)
        if not self.finite_beams:
            self.invalid += 1
            return
        before_live = self.live.copy()
        start = time.monotonic()
        try:
            valid = self.process_scan(filtered, tf)
            self.max_ray_ms = max(self.max_ray_ms, (time.monotonic()-start)*1000)
            if not valid:
                self.live = before_live
                self.invalid += 1
                return
            if not -0.05 <= self.scan_age(scan) < 0.6:
                self.live = before_live
                self.slow += 1
                return
            merged = copy.deepcopy(self.map_msg)
            values = np.asarray(merged.data, dtype=np.int16).reshape(self.live.shape).copy()
            values[self.live] = 100
            merged.data = values.ravel().tolist()
            merged.header.stamp = scan.header.stamp
            publish_start = time.monotonic()
            PomonaCostmap.on_map(self, merged)
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
        try:
            self.work()
        except (ValueError, RuntimeError, OverflowError) as exc:
            self.invalid += 1
            self.last_success_wall = None
            if time.monotonic()-self.last_warning >= 3.0:
                self.get_logger().warning(str(exc))
                self.last_warning = time.monotonic()
        wall = time.monotonic()
        source_age = math.inf if self.last_success_stamp is None else (
            self.get_clock().now()-self.last_success_stamp).nanoseconds*1e-9
        ready = (self.last_success_wall is not None
                 and wall-self.last_success_wall < 0.6 and -0.05 <= source_age < 0.6)
        self.ready_pub.publish(Bool(data=ready))
        if wall-self.last_diag >= 5.0:
            rx_gap = math.inf if self.last_rx_wall is None else wall-self.last_rx_wall
            self.get_logger().info(
                f'DIAG rx={self.rx} ok={self.ok} old={self.old} future={self.future} '
                f'tf_wait_attempts={self.tf_wait} invalid={self.invalid} slow={self.slow} '
                f'overflow={self.overflow} queue={len(self.scans)} '
                f'last_rx_age={self.last_rx_age:.3f}s rx_gap_now={rx_gap:.3f}s '
                f'max_rx_gap={self.max_rx_gap:.3f}s source_age={source_age:.3f}s '
                f'ray_max={self.max_ray_ms:.1f}ms inflate_publish_max={self.max_publish_ms:.1f}ms '
                f'finite_beams={self.finite_beams} ready={ready}')
            self.last_diag = wall


def main():
    rclpy.init()
    node = RealCostmapV2()
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
