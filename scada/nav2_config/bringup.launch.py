"""
bringup.launch.py — real-hardware SCADA full stack launch.

Launch order (sequence matters — EKF must publish /tf before Nav2 starts):
  1. robot_localization EKF  (publishes map→odom→base_footprint TF chain)
  2. Nav2 bringup            (needs TF tree to be valid on startup)
  3. vision_node             (overhead phone camera → /robot_pose)
  4. udp_bridge_node         (ESP32 UDP ↔ ROS2 topics)
  5. control_node            (mission FSM)
  6. bot_cam_node            (S3 MJPEG → /box_aruco_detected)

gui_node is NOT launched here — if you use bringup.launch.py, start GUI with:
  uv run scada/gui_node.py
Note: Prefer running everything in a single terminal with:
  uv run scada/main.py
(main.py embeds control_node, udp_bridge, vision_node, bot_cam, and the GUI).

Usage:
  source /opt/ros/humble/setup.bash
  ros2 launch scada/nav2_config/bringup.launch.py
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SCADA_DIR    = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
NAV2_CFG_DIR = os.path.dirname(__file__)

sys.path.insert(0, SCADA_DIR)
from arena_config import ARENA_FREE


# The scada/ nodes are plain Python files (not an installed ament package), so they are
# launched by running the file directly under the project's uv venv — the same environment
# that `uv run scada/main.py` uses. cwd=SCADA_DIR puts sibling imports (arena_config) on path.
def scada_proc(filename):
    return ExecuteProcess(
        cmd=["uv", "run", "python", filename],
        cwd=SCADA_DIR,
        output="screen",
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    # ── robot_localization EKF ───────────────────────────────────────────────
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(NAV2_CFG_DIR, "ekf.yaml"),
            {"use_sim_time": False},
        ],
        remappings=[("odometry/filtered", "/odom_ekf")],
    )

    # ── Static map→odom identity ─────────────────────────────────────────────
    # The single EKF (world_frame:odom) owns odom→base_footprint. Nothing else
    # publishes map→odom, so Nav2's global costmap (map frame) has no link to the
    # bot. map and odom coincide at start — the camera /robot_pose gives absolute
    # correction inside the EKF — so an identity static transform completes the
    # map→odom→base_footprint chain Nav2 requires.
    static_map_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_odom",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen",
    )

    # ── Nav2 bringup — delayed 1 s so EKF has time to publish initial TF ────
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
            "params_file":  os.path.join(NAV2_CFG_DIR, "nav2_params.yaml"),
            "autostart":    "true",
        }.items(),
    )
    nav2_delayed = TimerAction(period=1.0, actions=[nav2_launch])

    # ── SCADA Python nodes ───────────────────────────────────────────────────
    vision_node     = scada_proc("vision_node.py")
    udp_bridge_node = scada_proc("udp_bridge_node.py")
    control_node    = scada_proc("control_node.py")
    bot_cam_node    = scada_proc("bot_cam.py")

    # Arena-free mode navigates by pixel-space visual servoing — no EKF, no Nav2, no
    # map→odom TF. Launch only the camera, UDP bridge, FSM and bot-cam. (The Nav2 stack
    # is what emitted the "Robot is out of bounds of the costmap!" spam.)
    if ARENA_FREE:
        return LaunchDescription([
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            vision_node,
            udp_bridge_node,
            control_node,
            bot_cam_node,
        ])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        static_map_odom,
        ekf_node,
        nav2_delayed,
        vision_node,
        udp_bridge_node,
        control_node,
        bot_cam_node,
    ])
