#!/usr/bin/env python3
"""Preview a stand-off goal; send it ONCE only through /pomona/aruco/commit_goal.

Input: point_camera from pomona_aruco_real.py (ID 0, 10 cm by default).
Run on the laptop with the existing live costmap and RRT. Start follower
before committing a goal. This node never publishes velocities.

After commit the destination remains fixed even if the marker disappears.
Stopping this node does NOT cancel the planner or stop the follower.
Stop the follower/use manual control to interrupt navigation. Restart this
node between missions; it intentionally has no automatic rearming.
Stand-off is horizontal centre-to-marker distance, along the initial
robot-to-marker line, NOT marker-normal docking or bumper clearance.
"""
import math
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException


def quaternion_matrix(q):
    values = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(values)
    if not np.isfinite(norm) or norm < 1e-9:
        raise ValueError('Invalid TF quaternion')
    x, y, z, w = values / norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def apply_transform(tf, xyz):
    t = tf.translation
    result = quaternion_matrix(tf.rotation) @ xyz + np.array([t.x, t.y, t.z])
    if not np.all(np.isfinite(result)):
        raise ValueError('Nonfinite transformed point')
    return result


def stand_off_goal(marker_xy, robot_xy, distance):
    delta = np.asarray(marker_xy, dtype=float) - np.asarray(robot_xy, dtype=float)
    length = float(np.linalg.norm(delta))
    if not math.isfinite(distance) or distance <= 0:
        raise ValueError('Stand-off must be positive')
    if not math.isfinite(length) or length <= distance + 0.20:
        raise ValueError('Marker too close for this stand-off; move marker farther away')
    goal = marker_xy - distance * delta / length
    return goal, math.atan2(delta[1], delta[0])


class GoalBridge(Node):
    def __init__(self):
        super().__init__('pomona_aruco_goal_real')
        if self.get_parameter('use_sim_time').value:
            raise ValueError('This node requires use_sim_time=false')
        self.declare_parameter('stand_off_m', 0.80)
        self.standoff = float(self.get_parameter('stand_off_m').value)
        if not math.isfinite(self.standoff) or self.standoff < 0.70:
            raise ValueError('Initial real-robot tests require stand_off_m >= 0.70')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.samples = deque(maxlen=30)
        self.pending = deque(maxlen=20)
        self.last_processed_stamp = None
        self.map_msg = None
        self.map_wall = -math.inf
        self.ready = False
        self.ready_wall = -math.inf
        self.committed = False
        self.last_state = None
        self.last_log_wall = -math.inf
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.point_sub = self.create_subscription(PointStamped, '/pomona/aruco/point_camera', self.on_point, 1)
        self.map_sub = self.create_subscription(OccupancyGrid, '/pomona/inflated_map', self.on_map, qos)
        self.ready_sub = self.create_subscription(Bool, '/pomona/costmap_ready', self.on_ready, 1)
        self.preview_pub = self.create_publisher(PoseStamped, '/pomona/aruco/goal_preview', 1)
        self.marker_pub = self.create_publisher(PointStamped, '/pomona/aruco/point_map', 1)
        self.preview_ready_pub = self.create_publisher(Bool, '/pomona/aruco/goal_preview_ready', 1)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 1)
        self.service = self.create_service(Trigger, '/pomona/aruco/commit_goal', self.commit)
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.1, self.update, clock=self.steady)
        self.get_logger().info('PREVIEW ONLY v2 (timestamped TF queue). Stable marker + fresh free costmap required. Commit service sends ONE goal.')

    def report(self, text):
        wall = time.monotonic()
        if text != self.last_state and wall - self.last_log_wall >= 2.0:
            self.get_logger().info(text)
            self.last_state = text
            self.last_log_wall = wall

    def age(self, stamp):
        return (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds * 1e-9

    def on_point(self, msg):
        # New observations must not overwrite observations still awaiting TF.
        self.pending.append(msg)

    def on_map(self, msg):
        self.map_msg = msg
        self.map_wall = time.monotonic()

    def on_ready(self, msg):
        self.ready = msg.data
        self.ready_wall = time.monotonic()

    def consume_point(self):
        # Keep only fresh, strictly newer observations. Try newest first, but
        # use an older fresh observation if its timestamped TF is ready.
        # Never substitute latest TF for the image-time TF, and never block
        # the executor while the same executor needs to receive TF messages.
        fresh = deque(maxlen=20)
        for item in self.pending:
            key = (item.header.stamp.sec, item.header.stamp.nanosec)
            if (self.last_processed_stamp is None or key > self.last_processed_stamp) and -0.05 <= self.age(item.header.stamp) < 0.5:
                fresh.append(item)
        self.pending = fresh
        selected = None
        for msg in reversed(self.pending):
            point = np.array([msg.point.x, msg.point.y, msg.point.z])
            if not msg.header.frame_id or not np.all(np.isfinite(point)):
                continue
            stamp = Time.from_msg(msg.header.stamp)
            try:
                camera_tf = self.buffer.lookup_transform('map', msg.header.frame_id, stamp).transform
                robot_tf = self.buffer.lookup_transform('map', 'base_link', stamp).transform
            except TransformException:
                continue
            selected = (msg, point, camera_tf, robot_tf)
            break
        if selected is None:
            return
        msg, point, camera_tf, robot_tf = selected
        stamp_key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if not -0.05 <= self.age(msg.header.stamp) < 0.5:
            return
        marker = apply_transform(camera_tf, point)
        robot = apply_transform(robot_tf, np.zeros(3))
        rotation = quaternion_matrix(robot_tf.rotation)
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        wall = time.monotonic()
        if self.samples and wall - self.samples[-1][0] > 0.5:
            self.samples.clear()
        self.samples.append((wall, marker, robot, yaw, msg.header.stamp))
        self.last_processed_stamp = stamp_key
        self.pending = deque((item for item in self.pending
                              if (item.header.stamp.sec, item.header.stamp.nanosec) > stamp_key),
                             maxlen=20)
        out = PointStamped()
        out.header.frame_id = 'map'
        out.header.stamp = msg.header.stamp
        out.point.x, out.point.y, out.point.z = map(float, marker)
        self.marker_pub.publish(out)

    def free_goal(self, xy):
        msg = self.map_msg
        wall = time.monotonic()
        if (msg is None or not self.ready or wall-self.ready_wall >= 0.6
                or wall-self.map_wall >= 0.6):
            source_age = 'none' if msg is None else f'{self.age(msg.header.stamp):.3f}s'
            raise ValueError(f'Waiting for fresh costmap/status: map_receive_gap={wall-self.map_wall:.3f}s, map_source_age={source_age}, ready={self.ready}, ready_gap={wall-self.ready_wall:.3f}s')
        if msg.header.frame_id != 'map':
            raise ValueError(f'Costmap frame must be map, received {msg.header.frame_id}')
        if not -0.05 <= self.age(msg.header.stamp) < 0.6:
            raise ValueError(f'Costmap source stale/future: age={self.age(msg.header.stamp):.3f}s')
        info = msg.info
        if (not math.isfinite(info.resolution) or info.resolution <= 0
                or info.width == 0 or info.height == 0
                or len(msg.data) != info.width*info.height):
            raise ValueError('Invalid costmap geometry')
        rot = quaternion_matrix(info.origin.orientation)
        if abs(rot[2, 2]-1) > 1e-6:
            raise ValueError('Nonplanar costmap origin')
        p = info.origin.position
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            raise ValueError('Invalid costmap origin')
        local = rot.T @ (np.array([xy[0], xy[1], p.z])-np.array([p.x,p.y,p.z]))
        gx, gy = np.floor(local[:2]/info.resolution).astype(int)
        if not (0 <= gx < info.width and 0 <= gy < info.height):
            raise ValueError('Goal outside costmap')
        if msg.data[gy*info.width+gx] != 0:
            raise ValueError('Goal occupied/inflated/unknown; reposition marker, do not reduce inflation')

    def candidate(self):
        if self.committed:
            raise ValueError('Goal already sent; fixed destination retained by RRT')
        wall = time.monotonic()
        while self.samples and wall-self.samples[0][0] > 3.0:
            self.samples.popleft()
        if len(self.samples) < 6 or self.samples[-1][0]-self.samples[0][0] < 1.0:
            raise ValueError('Collecting >=6 samples over >=1 second; keep robot and marker still')
        last = self.samples[-1]
        if wall-last[0] >= 0.5 or not -0.05 <= self.age(last[4]) < 0.5:
            raise ValueError('Marker observation stale')
        markers = np.array([s[1] for s in self.samples])
        robots = np.array([s[2] for s in self.samples])
        marker = np.median(markers, axis=0)
        robot = np.median(robots, axis=0)
        if np.max(np.linalg.norm(markers-marker, axis=1)) > 0.08:
            raise ValueError('Marker/map position unstable; hold still and check localization')
        if np.max(np.linalg.norm(robots[:, :2]-robot[:2], axis=1)) > 0.05:
            raise ValueError('Robot moving or localization unstable')
        yaw = last[3]
        if any(abs(math.atan2(math.sin(s[3]-yaw),math.cos(s[3]-yaw))) > 0.05 for s in self.samples):
            raise ValueError('Robot heading changed; hold still')
        current = self.buffer.lookup_transform('map', 'base_link', Time())
        if not -0.05 <= self.age(current.header.stamp) < 0.6:
            raise ValueError('Robot TF stale')
        pose = apply_transform(current.transform, np.zeros(3))
        rotation = quaternion_matrix(current.transform.rotation)
        now_yaw = math.atan2(rotation[1,0],rotation[0,0])
        if (np.linalg.norm(pose[:2]-robot[:2]) > 0.05
                or abs(math.atan2(math.sin(now_yaw-yaw),math.cos(now_yaw-yaw))) > 0.05):
            raise ValueError('Robot moved since observations; hold still')
        xy, heading = stand_off_goal(marker[:2], robot[:2], self.standoff)
        self.free_goal(xy)
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = map(float, xy)
        goal.pose.orientation.z = math.sin(heading/2)
        goal.pose.orientation.w = math.cos(heading/2)
        return goal

    def update(self):
        try:
            if self.committed:
                self.preview_ready_pub.publish(Bool(data=False))
                return
            self.consume_point()
            goal = self.candidate()
            self.preview_pub.publish(goal)
            self.preview_ready_pub.publish(Bool(data=True))
            self.report('Preview READY: verify RViz. Commit service sends one /goal_pose.')
        except (ValueError, TransformException) as exc:
            self.preview_ready_pub.publish(Bool(data=False))
            self.report(str(exc))

    def commit(self, request, response):
        try:
            self.consume_point()
            goal = self.candidate()
            if self.goal_pub.get_subscription_count() == 0:
                raise ValueError('No /goal_pose subscriber; start RRT first')
            self.goal_pub.publish(goal)
            self.committed = True
            response.success = True
            response.message = f'Sent ONE map goal: x={goal.pose.position.x:.3f}, y={goal.pose.position.y:.3f}; standoff={self.standoff:.2f}m. Marker loss does not cancel this goal.'
            self.report(response.message)
        except (ValueError, TransformException) as exc:
            response.success = False
            response.message = str(exc)
        return response


def main():
    rclpy.init()
    node = GoalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
