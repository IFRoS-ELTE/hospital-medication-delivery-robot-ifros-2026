from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import ReplaceString


def generate_launch_description():
    workspace = Path(__file__).resolve().parents[1]
    config_dir = workspace / "config"

    localization_params = ReplaceString(
        source_file=str(config_dir / "pomona_localization.yaml"),
        replacements={
            "__POMONA_MAP_PREFIX__": str(
                workspace / "maps" / "pomona_room_v1"
            ),
        },
    )

    world_file = ReplaceString(
        source_file=str(
            workspace / "vendor_overrides" / "scout"
            / "scout_gazebo_sim" / "worlds"
            / "pomona_mapping_room.sdf"
        ),
        replacements={
            "__POMONA_ARUCO_MODEL_URI__": (
                workspace / "models" / "pomona_aruco_0" / "model.sdf"
            ).as_uri(),
        },
    )

    gazebo_launch = (
        Path(get_package_share_directory("scout_gazebo_sim"))
        / "launch"
        / "scout_mini_empty_world.launch.py"
    )

    localization_launch = (
        Path(get_package_share_directory("slam_toolbox"))
        / "launch"
        / "localization_launch.py"
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_launch)),
        launch_arguments={
            "world_name": world_file,
            "use_rviz": "false",
            "use_sim_time": "true",
        }.items(),
    )

    localization = GroupAction(
        actions=[
            SetRemap(
                src="/tf",
                dst="/scout_mini/tf",
            ),
            SetRemap(
                src="/tf_static",
                dst="/scout_mini/tf_static",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(localization_launch)),
                launch_arguments={
                    "slam_params_file": localization_params,
                    "use_sim_time": "true",
                    "autostart": "true",
                    "use_lifecycle_manager": "false",
                }.items(),
            ),
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="pomona_rviz",
        output="screen",
        arguments=[
            "-d",
            str(config_dir / "pomona_navigation.rviz"),
        ],
        parameters=[{"use_sim_time": True}],
        remappings=[
            ("/tf", "/scout_mini/tf"),
            ("/tf_static", "/scout_mini/tf_static"),
        ],
    )

    return LaunchDescription([
        simulation,
        localization,
        rviz,
    ])