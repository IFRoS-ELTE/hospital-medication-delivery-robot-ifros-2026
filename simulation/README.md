# Pomona Simulation Pipeline

ROS 2 Jazzy and Gazebo simulation for ArUco-guided delivery navigation.

## Components

- Saved-map localization using SLAM Toolbox.
- Static obstacle inflation and a live LiDAR costmap layer.
- RRT path planning with replanning when the remaining path is blocked.
- Waypoint following with final-heading alignment.
- ArUco detection, transformation into the map frame, and goal generation.

Frontier exploration is not included in this version.

## Directory layout

| Directory | Contents |
|---|---|
| scripts/ | Perception, costmap, planning and control nodes. |
| launch/ | Localization and complete navigation launches. |
| config/ | SLAM Toolbox and RViz configuration. |
| maps/ | Occupancy map and serialized SLAM pose graph. |
| models/ | ArUco marker model. |
| vendor_overrides/ | Modified robot description and simulation worlds. |

## Environment

Tested on Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Sim.

The simulation depends on `ugv_gazebo_sim`. Its source URL and base
commit are recorded in `UPSTREAM.txt`.

The modified `scout_mini.xacro` adds the simulated LiDAR and RGB camera.
Apply the files in `vendor_overrides/scout/` to the corresponding
`scout/` directory in the upstream checkout, then rebuild the required
ROS packages.

Additional dependencies include SLAM Toolbox, Nav2 Common, NumPy,
OpenCV with ArUco support, and cv_bridge.

## Run with the existing simulation workspace

The following command was tested with the prepared `~/pomona_ws`
workspace. That workspace must contain the modified Scout description
and the required Gazebo bridge packages.

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
source ~/pomona_ws/install/setup.bash
export ROS_DOMAIN_ID=0

ros2 launch \
  "$PWD/simulation/launch/pomona_sim_navigation.launch.py"
```

Project scripts, configuration, map and ArUco model are resolved from
this repository. Machine-specific map and model paths are generated
at launch time.

Do not run another simulation instance concurrently.

## Send an ArUco goal

In another terminal:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0

ros2 topic echo /pomona/costmap_ready --once
```

After the costmap is ready and the marker is detected:

```bash
ros2 service call /pomona/aruco/send_goal std_srvs/srv/Trigger "{}"
```

Goals can also be selected using 2D Goal Pose in RViz.

Stop the complete launch with Ctrl+C.

## Validation status

The original simulation demonstrated waypoint navigation, obstacle-triggered
replanning, ArUco goal generation, and final-heading alignment.

After relocation into this repository, the launch was reported to run
successfully and SLAM Toolbox loaded the repository-local pose graph.

A clean installation on a second machine has not yet been verified.
This simulation does not establish autonomous-navigation performance
on the physical robot.
