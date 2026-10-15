"""
bringup.launch.py — real-hardware SCADA full stack launch.

Launch order (sequence matters — EKF must publish /tf before Nav2 starts):
  1. robot_localization EKF  (publishes map→odom→base_footprint TF chain)
  2. Nav2 bringup            (needs TF tree to be valid on startup)
  3. vision_node             (overhead phone camera → /robot_pose)
  4. udp_bridge_node         (ESP32 UDP ↔ ROS2 topics)
  5. control_node            (mission FSM)
  6. bot_cam_node            (S3 MJPEG → /box_aruco_detected)

gui_node is NOT launched here — start it separately with:
  uv run scada/main.py
or add it below if you want everything in one command.

Usage:
  source /opt/ros/humble/setup.bash
  ros2 launch scada/nav2_config/bringup.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SCADA_DIR    = os.path.join(os.path.dirname(__file__), "..")
NAV2_CFG_DIR = os.path.dirname(__file__)


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

    # ── Nav2 bringup — delayed 1 s so EKF has time to publish initial TF ────
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time":  "false",
            "params_file":   os.path.join(NAV2_CFG_DIR, "nav2_params.yaml"),
            "map":           "",              # no pre-built map; /map published by vision_node
            "autostart":     "true",
        }.items(),
    )
    nav2_delayed = TimerAction(period=1.0, actions=[nav2_launch])

    # ── SCADA Python nodes ───────────────────────────────────────────────────
    vision_node = Node(
        package="warden_scada",
        executable="vision_node",
        name="vision_node",
        output="screen",
        # Fallback: run directly if not installed as a package
        # python3 scada/vision_node.py
    )

    udp_bridge_node = Node(
        package="warden_scada",
        executable="udp_bridge_node",
        name="udp_bridge_node",
        output="screen",
    )

    control_node = Node(
        package="warden_scada",
        executable="control_node",
        name="control_node",
        output="screen",
    )

    bot_cam_node = Node(
        package="warden_scada",
        executable="bot_cam_node",
        name="bot_cam_node",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        ekf_node,
        nav2_delayed,
        vision_node,
        udp_bridge_node,
        control_node,
        bot_cam_node,
    ])
