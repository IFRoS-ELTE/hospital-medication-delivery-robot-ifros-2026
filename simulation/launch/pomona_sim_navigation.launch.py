"""Start the tested Pomona simulation and navigation nodes together.

Run after sourcing ROS Jazzy and the pomona_ws install overlay.
ArUco goals remain manual: call /pomona/aruco/send_goal when ready.
"""

import ast
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    workspace = Path(__file__).resolve().parents[1]
    scripts_dir = workspace / "scripts"
    base_launch = workspace / "launch" / "pomona_sim_localization.launch.py"
    script_names = [
        "pomona_live_costmap.py",
        "pomona_aruco.py",
        "pomona_aruco_map.py",
        "pomona_aruco_goal.py",
        "pomona_rrt.py",
        "pomona_follower.py",
    ]

    # Validate all local inputs before starting any process.
    required_python = [base_launch] + [
        scripts_dir / name for name in script_names
    ] + [scripts_dir / "pomona_costmap.py"]
    required_files = required_python + [
        workspace / "config" / "pomona_localization.yaml",
        workspace / "config" / "pomona_navigation.rviz",
    ]
    missing = [str(path) for path in required_files if not path.is_file()]
    if missing:
        raise RuntimeError("Missing Pomona files:\n" + "\n".join(missing))
    for path in required_python:
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))

    simulation_and_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(base_launch))
    )

    processes = []
    handlers = []
    for script_name in script_names:
        process = ExecuteProcess(
            cmd=[
                "python3", "-u", str(scripts_dir / script_name),
                "--ros-args",
                "-p", "use_sim_time:=true",
                "-r", "/tf:=/scout_mini/tf",
                "-r", "/tf_static:=/scout_mini/tf_static",
            ],
            name=Path(script_name).stem,
            cwd=str(scripts_dir),
            output="screen",
            # Allow the follower to send zero velocity on Ctrl+C.
            sigterm_timeout="5",
            sigkill_timeout="5",
        )
        processes.append(process)
        # If a navigation component exits, stop the whole launch rather
        # than leaving the remaining components running on their own.
        handlers.append(RegisterEventHandler(
            OnProcessExit(
                target_action=process,
                on_exit=[
                    EmitEvent(event=Shutdown(
                        reason=f"Pomona component exited: {script_name}"
                    )),
                ],
            )
        ))

    return LaunchDescription([
        LogInfo(msg=(
            "Starting Pomona simulation, localization, RViz, live costmap, "
            "ArUco detection, marker transform, goal service, RRT and follower. "
            "Wait for costmap_ready=true, then call /pomona/aruco/send_goal."
        )),
        *handlers,
        simulation_and_localization,
        *processes,
    ])
