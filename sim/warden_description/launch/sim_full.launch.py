"""
sim_full.launch.py — Tier-1 full Gazebo simulation.

Single command to start everything:
  cd ~/ros2_ws && colcon build --packages-select warden_description
  source install/setup.bash
  ros2 launch warden_description sim_full.launch.py

What runs:
  [0s]  gzserver — Gazebo physics + arena.world
  [4s]  gzclient — Gazebo GUI
  [4s]  robot_state_publisher — URDF → /tf
  [4s]  static map→odom TF (identity; Gazebo odom IS ground truth)
  [6s]  sim_inspection_node — /map + /tof_range + repair sim
  [9s]  Nav2 (planner + controller + recoveries)
  [11s] control_node — FSM + Nav2 goal client
  [11s] gui_node — PyQt5 dashboard
  [13s] spawn_entity — bot into Gazebo

Nodes NOT launched here (not needed in sim):
  udp_bridge_node  (talks to real ESP32-C6 — skip in sim)
  vision_node      (reads phone MJPEG — skip in sim)
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess, IncludeLaunchDescription,
    SetEnvironmentVariable, TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

SCADA_DIR  = "/home/xio/Desktop/warden/scada"
SIM_DIR    = "/home/xio/Desktop/warden/sim"
NAV2_CFG   = os.path.join(SCADA_DIR, "nav2_config", "nav2_params.yaml")
VENV_PY    = "/home/xio/Desktop/warden/.venv/bin/python3"
SIM_VISION = os.path.join(SIM_DIR, "sim_vision_node.py")


def generate_launch_description():
    pkg_name   = "warden_description"
    pkg_path   = get_package_share_directory(pkg_name)
    gazebo_pkg = get_package_share_directory("gazebo_ros")
    nav2_pkg   = get_package_share_directory("nav2_bringup")

    world_path = os.path.join(pkg_path, "worlds", "arena.world")

    # ── Env vars Gazebo needs ────────────────────────────────────────────────
    disable_db = SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", "")

    existing_mp = os.environ.get("GAZEBO_MODEL_PATH", "")
    fix_mp = SetEnvironmentVariable(
        "GAZEBO_MODEL_PATH",
        os.path.dirname(pkg_path) + ":/usr/share/gazebo-11/models"
        + (":" + existing_mp if existing_mp else ""),
    )

    existing_rp = os.environ.get("GAZEBO_RESOURCE_PATH", "")
    rp_parts    = [pkg_path]
    if existing_rp:
        rp_parts.append(existing_rp)
    if "/usr/share/gazebo-11" not in existing_rp:
        rp_parts.append("/usr/share/gazebo-11")
    fix_rp = SetEnvironmentVariable("GAZEBO_RESOURCE_PATH", ":".join(rp_parts))

    # ── URDF ────────────────────────────────────────────────────────────────
    xacro_file = os.path.join(pkg_path, "models", "urdf", "warden.urdf.xacro")
    urdf_str   = xacro.process_file(xacro_file).toxml().replace(
        f"package://{pkg_name}/", f"file://{pkg_path}/"
    )

    # ── Nodes ───────────────────────────────────────────────────────────────

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"verbose": "true", "gui": "false", "world": world_path}.items(),
    )

    gzclient = TimerAction(period=4.0, actions=[
        ExecuteProcess(cmd=["gzclient"], output="screen")
    ])

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": urdf_str, "use_sim_time": True}],
        output="screen",
    )

    # Static map→odom identity transform: Gazebo diff_drive gives exact odom,
    # so map≡odom for Tier-1 (no localization drift to correct).
    map_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--roll", "0", "--pitch", "0", "--yaw", "0",
                   "--frame-id", "map", "--child-frame-id", "odom"],
        parameters=[{"use_sim_time": True}],
    )

    overhead_cam_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="overhead_cam_tf",
        arguments=["--x", "1.0", "--y", "1.0", "--z", "3.0",
                   "--roll", "0", "--pitch", "1.5708", "--yaw", "0",
                   "--frame-id", "odom", "--child-frame-id", "overhead_camera_link"],
        parameters=[{"use_sim_time": True}],
    )

    # sim_inspection_node: /map + /tof_range + repair responses (no longer owns /robot_pose)
    sim_inspection = TimerAction(period=6.0, actions=[
        ExecuteProcess(
            cmd=[VENV_PY, os.path.join(SIM_DIR, "sim_inspection_node.py")],
            output="screen",
        )
    ])

    # sim_vision_node: real ArUco detection from Gazebo overhead camera → /robot_pose
    sim_vision = TimerAction(period=7.0, actions=[
        ExecuteProcess(
            cmd=[VENV_PY, SIM_VISION],
            output="screen",
        )
    ])

    nav2 = TimerAction(period=9.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_pkg, "launch", "navigation_launch.py")
            ),
            launch_arguments={
                "use_sim_time": "true",
                "params_file":  NAV2_CFG,
                "autostart":    "true",
            }.items(),
        )
    ])

    control_node = TimerAction(period=11.0, actions=[
        ExecuteProcess(
            cmd=[VENV_PY, os.path.join(SCADA_DIR, "control_node.py")],
            output="screen",
        )
    ])

    gui_node = TimerAction(period=11.0, actions=[
        ExecuteProcess(
            cmd=[VENV_PY, os.path.join(SCADA_DIR, "gui_node.py"), "--sim"],
            output="screen",
        )
    ])

    spawn = TimerAction(period=13.0, actions=[
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-topic", "robot_description",
                       "-entity", "warden",
                       "-x", "0.1", "-y", "0.1", "-z", "0.05"],
            output="screen",
        )
    ])

    return LaunchDescription([
        disable_db, fix_mp, fix_rp,
        gzserver, rsp, map_odom_tf, overhead_cam_tf,
        gzclient,
        sim_inspection,
        sim_vision,
        nav2,
        control_node, gui_node,
        spawn,
    ])
