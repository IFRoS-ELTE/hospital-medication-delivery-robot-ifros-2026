# Pomona Real-Robot Perception and Navigation

ROS 2 Python nodes used for the hospital medication delivery project on
the physical AgileX Scout Mini (Pomona). The supplied eight Python files
are preserved byte-for-byte from the robot export of 28 October 2026.

The pipeline detects an ArUco delivery marker, creates a fixed stand-off
goal, plans with RRT, and follows waypoints while checking the live costmap.
Obstacle handling is reactive stopping and replanning; obstacle trajectories
are not predicted. Real-robot demonstrations were reported for an unobstructed
approach, a static obstacle, and an obstacle introduced during navigation.
These are qualitative demonstrations, not a repeated-trial reliability study.

## Files and Dependencies

| File | Purpose |
|---|---|
| `pomona_aruco_real.py` | Detects the configured marker and publishes timestamped positions in camera and robot frames. |
| `nav_real/pomona_aruco_goal_real.py` | Validates stable observations and fresh costmap data; previews and commits one map goal. |
| `nav_real/pomona_rrt.py` | RRT planning, remaining-path validation, replanning and final-heading completion checks. |
| `nav_real/pomona_follower.py` | Waypoint tracking, remaining-path checks and velocity output. Imports collision utilities from `pomona_rrt.py`. |
| `nav_real/pomona_live_costmap_real_v3.py` | Active live costmap entry point; batched ray processing and binary occupancy data. |
| `nav_real/pomona_live_costmap_real_v2.py` | Scan-time TF queue, freshness checks and diagnostics inherited by v3. |
| `nav_real/pomona_live_costmap_real.py` | Base live-layer implementation and quaternion utility imported by v2/v3. |
| `nav_real/pomona_costmap.py` | OpenCV inflation backend and base map subscription. |

Keep all seven `nav_real/` files together. Run **only v3** as the live
costmap process; v2 and the original live module remain import dependencies.
The goal bridge's original module comment mentions laptop execution; the
deployment documented here runs navigation and perception on the robot.

## Runtime and External Assets

The tested setup uses the existing ARM64 image `pomona:jazzy-20260918`
on Xavier, with ROS 2 Jazzy, NumPy, OpenCV 4.6.0 with `cv2.aruco`,
`cv_bridge`, `tf2_ros`, and standard ROS message/service packages.
Inflation uses OpenCV and does not require SciPy.

The detector uses the OpenCV 4.6 APIs `DetectorParameters_create`,
`detectMarkers` and `estimatePoseSingleMarkers`. Compatibility with newer
OpenCV releases has not been verified. Use the tested container environment
rather than replacing its OpenCV installation with a different pip build.

This source export does **not** include the Docker image/build recipe,
hardware bringup, URDF/mounting geometry, localization configuration, or maps.
It assumes the existing robot deployment provides:

- Container `pomona` with `/data/pomona` mounted at `/bags`.
- `/usr/local/bin/pomona-entrypoint` to source the ROS environment.
- `~/pomona-transfer/pomona-payload/pomona-up.sh` on the robot host.
- `/data/pomona/config/pomona_real_localization_20261008.yaml`.
- Serialized map files for `/bags/maps/pomona_real_room_20261008_v1`.
- Correct physical camera/LiDAR mounting transforms and camera calibration.

`ROS_DOMAIN_ID=43` and `use_sim_time=false` are used throughout. The robot
and laptop clocks must be synchronized before starting ROS nodes.

## Deployment to the Existing Robot

From the repository root on the laptop, stage the source in the robot's home:

```bash
scp -r real_robot agilex@192.168.1.102:~/pomona_real_source_20261028
```

On Xavier, with navigation stopped, back up existing files before replacing
them. The following commands expect the staging directory above:

```bash
backup_dir="/data/pomona/config/backup_$(date +%Y%m%d_%H%M%S)"
sudo mkdir -p "$backup_dir"
sudo cp -a /data/pomona/config/nav_real "$backup_dir/"
sudo cp -a /data/pomona/config/pomona_aruco_real.py "$backup_dir/"
sudo cp -a ~/pomona_real_source_20261028/nav_real /data/pomona/config/
sudo cp ~/pomona_real_source_20261028/pomona_aruco_real.py /data/pomona/config/
```

There is no need to reinstall the unchanged export on the already working
robot merely to publish it to GitHub.

Check the existing container dependencies:

```bash
sudo docker exec -it pomona /usr/local/bin/pomona-entrypoint \
  python3 -c 'import cv2, numpy, rclpy; from cv_bridge import CvBridge; print("OpenCV:", cv2.__version__); print("ArUco legacy API:", all(hasattr(cv2.aruco, name) for name in ("DetectorParameters_create", "detectMarkers", "estimatePoseSingleMarkers")))'
```

## Startup

Use a separate Xavier SSH terminal for each long-running command. Keep the
robot stationary during initialization, and have an operator at the emergency
stop during motion tests. A successful commit can start robot movement.

### 1. Hardware bringup

After clock synchronization, configure CAN and launch the existing stack:

```bash
sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can0 up
cd ~/pomona-transfer/pomona-payload
sudo ./pomona-up.sh --base
```

This launcher enables the base, LiDAR and camera. Keep its terminal running.

### 2. Localization

```bash
sudo docker exec -it pomona /usr/local/bin/pomona-entrypoint \
  ros2 launch slam_toolbox localization_launch.py \
  slam_params_file:=/bags/config/pomona_real_localization_20261008.yaml \
  use_sim_time:=false
```

On the laptop, RViz can be used to check localization:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=43
rviz2
```

Set Fixed Frame to `map`, display `/map` with transient-local durability,
and `/scan` with best-effort reliability. Use **2D Pose Estimate** if the
robot pose is incorrect, then verify that scans align with fixed walls.
Closing RViz does not stop localization; keep the localization terminal alive.
Gazebo is not used for these tests.

### 3. Live costmap v3

```bash
sudo docker exec -it \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS \
  -e RMW_FASTRTPS_USE_QOS_FROM_XML=0 \
  pomona /usr/local/bin/pomona-entrypoint \
  python3 /bags/config/nav_real/pomona_live_costmap_real_v3.py \
  --ros-args -p use_sim_time:=false -r /scout_mini/scan:=/scan
```

Check `DIAG` output for fresh `source_age` and `ready=True`. Remote RViz
subscriptions to full maps previously increased transport latency; close
RViz after checking localization and the inflated map if this recurs.
Do not disable freshness checks to make the robot continue on old data.

### 4. ArUco detection and goal preview

Detector:

```bash
sudo docker exec -it pomona /usr/local/bin/pomona-entrypoint \
  python3 /bags/config/pomona_aruco_real.py
```

Goal bridge, in another terminal:

```bash
sudo docker exec -it pomona /usr/local/bin/pomona-entrypoint \
  python3 /bags/config/nav_real/pomona_aruco_goal_real.py
```

The default target is **ID 0**, dictionary **DICT_6X6_1000**, with a
**0.10 m marker side** excluding the surrounding white margin. Detection
is processed at up to 5 Hz. Keep the marker and robot stationary while the
bridge collects at least six samples spanning at least one second.

The bridge transforms observations at their timestamps into `map` and checks
stability, current robot pose, costmap freshness and free goal occupancy.
It defaults to a **0.80 m horizontal stand-off from the robot origin** along
the robot-to-marker line. This is not bumper clearance or marker-normal docking.

### 5. RRT and follower

Planner:

```bash
sudo docker exec -it pomona /usr/local/bin/pomona-entrypoint \
  python3 /bags/config/nav_real/pomona_rrt.py \
  --ros-args -p use_sim_time:=false
```

Follower, in another terminal:

```bash
sudo docker exec -it pomona /usr/local/bin/pomona-entrypoint \
  python3 /bags/config/nav_real/pomona_follower.py \
  --ros-args -p use_sim_time:=false -r /scout_mini/cmd_vel:=/cmd_vel
```

Check that the follower is the only intended `/cmd_vel` publisher:

```bash
sudo docker exec -it pomona /usr/local/bin/pomona-entrypoint \
  ros2 topic info /cmd_vel --verbose
```

### 6. Commit one goal

When the route is clear for the planned test, recording is running, and the
bridge reports `Preview READY`, request a goal:

```bash
sudo docker exec -it pomona /usr/local/bin/pomona-entrypoint \
  ros2 service call /pomona/aruco/commit_goal std_srvs/srv/Trigger '{}'
```

The service rechecks conditions at request time. `success=True` confirms
publication of one goal, not arrival. Check RRT and follower logs for progress
and `Goal reached: position and heading` at completion. Stale inputs or a
blocked route may cause stopping; readiness at commit does not guarantee a
feasible path or successful navigation.

After commit, marker loss does not cancel the fixed destination. Stopping
the detector or bridge also does not cancel navigation.

## Main Interfaces

| Interface | Type | Role |
|---|---|---|
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | Camera input. |
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | Calibration input. |
| `/scan` | `sensor_msgs/msg/LaserScan` | Real LiDAR scan input after remapping. |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Localization map input. |
| `/pomona/aruco/point_camera`, `/pomona/aruco/point_base` | `geometry_msgs/msg/PointStamped` | Timestamped marker positions. |
| `/pomona/aruco/detected` | `std_msgs/msg/Bool` | Fresh valid observation status. |
| `/pomona/aruco/point_map` | `geometry_msgs/msg/PointStamped` | Marker position in `map`. |
| `/pomona/aruco/goal_preview` | `geometry_msgs/msg/PoseStamped` | Candidate goal. |
| `/pomona/aruco/goal_preview_ready` | `std_msgs/msg/Bool` | Candidate currently passes checks. |
| `/pomona/aruco/commit_goal` | `std_srvs/srv/Trigger` | One-shot commit service. |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Committed map goal. |
| `/pomona/inflated_map` | `nav_msgs/msg/OccupancyGrid` | Merged inflated obstacle map. |
| `/pomona/costmap_ready`, `/pomona/path_valid` | `std_msgs/msg/Bool` | Costmap and path status. |
| `/pomona/rrt_path` | `nav_msgs/msg/Path` | Planned waypoints. |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Base command after follower remapping. |

Required TF includes `map -> odom -> base_footprint -> base_link` and the
sensor mounting/optical frames. Sensor-to-map transforms are looked up at
observation time; the nodes do not substitute a latest transform on failure.

## Defaults and Limitations

- Inflation: 0.43 m robot radius + 0.07 m margin, with an additional grid-cell
  allowance. Only known-free cells are traversable; unknown cells and the map
  boundary are blocked.
- Live v3 rays use finite returns below `range_max`. Infinite/no-return beams
  do not clear the live layer. Unobserved obstacle cells can therefore persist.
- Costmap readiness and follower map freshness use 0.6 s limits; the planner
  uses a 1.0 s map limit. Scan processing requires input younger than 0.5 s.
- Linear command is capped at 0.10 m/s and angular command at 0.30 rad/s.
- Final position/heading thresholds are 0.08 m and 0.05 rad; position correction
  resumes beyond 0.12 m. These are software thresholds, not measured accuracy.
- The detector requires matching image/CameraInfo geometry and frame, with
  `plumb_bob` distortion. Published old points are not erased on marker loss;
  consumers must check timestamps and status.

## Reset and Troubleshooting

| Message or symptom | Action |
|---|---|
| `Collecting >=6 samples...` | Hold robot and marker still; wait for a fresh stable observation window. |
| `Marker observation stale` | Check real camera rate, visibility, timestamps and timestamped TF. |
| `Waiting for fresh costmap/status` | Inspect costmap `DIAG`, `/scan`, TF and host clock synchronization. |
| `Goal occupied/inflated/unknown` | Choose a free destination and check localization; retain inflation. |
| `Goal already sent` | Bridge accepts one goal per process; start a new mission as described below. |
| Commit succeeds but robot stays still | Inspect planner/follower logs, free start/goal, fresh map/path/TF and `/cmd_vel` endpoints. Do not repeatedly recommit. |

For a new trial, first stop navigation safely, then stop the follower, planner
and goal bridge. Reposition only after motion is disabled, recheck localization
if needed, and restart the bridge, planner and follower before committing a
new goal. Keep hardware bringup, localization, costmap and detector running
if they remain healthy. Restarting the bridge alone does not clear the old
goal retained by the planner.
