import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess, IncludeLaunchDescription,
    SetEnvironmentVariable, TimerAction,
    DeclareLaunchArgument
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name   = 'warden_description'
    pkg_path   = get_package_share_directory(pkg_name)
    gazebo_pkg = get_package_share_directory('gazebo_ros')

    # world_path = '/usr/share/gazebo-11/worlds/empty.world'
    world_path = os.path.join(pkg_path, 'worlds', 'arena.world')
    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to the Gazebo world model file to load'
    )
    world_config = LaunchConfiguration('world')

    # ── Xacro → URDF, pre-resolve package:// → file:// ───────────────────
    # Gazebo converts package:// → model:// during URDF→SDF which breaks
    # model DB lookup. Replacing with file:// means Gazebo loads STLs
    # directly from disk — no model database lookup needed.
    xacro_file = os.path.join(pkg_path, 'models', 'urdf', 'warden.urdf.xacro')
    urdf_str   = xacro.process_file(xacro_file).toxml()
    urdf_str   = urdf_str.replace(
        f'package://{pkg_name}/',
        f'file://{pkg_path}/'
    )
    robot_description = {'robot_description': urdf_str}

    # ── Environment ────────────────────────────────────────────────────────
    # Disable blocking HTTP fetch to models.gazebosim.org on every startup.
    # Without this, gzserver hangs >30s and spawn_entity always times out.
    disable_online_db = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI', value=''
    )
    # Keep system Gazebo models (sun, ground_plane) findable.
    # Also add parent of pkg_path so model://warden_description/... resolves.
    system_models  = '/usr/share/gazebo-11/models'
    pkg_parent     = os.path.dirname(pkg_path)
    existing       = os.environ.get('GAZEBO_MODEL_PATH', '')
    fix_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=pkg_parent + ':' + system_models + (':' + existing if existing else '')
    )
    # Expose the package root so Gazebo Classic finds materials/scripts/*.material
    # (Gazebo searches <GAZEBO_RESOURCE_PATH>/materials/scripts/ for OGRE scripts)
    # Always include /usr/share/gazebo-11 — gzserver.launch.py builds its own
    # GAZEBO_RESOURCE_PATH from GazeboRosPaths.get_paths() which returns empty on
    # a bare ROS2 install (Gazebo Classic doesn't export a gazebo_media_path tag),
    # so if we don't inject the system path here gzserver can't find OGRE shaders
    # and gzclient crashes with a null Camera assertion.
    existing_rp = os.environ.get('GAZEBO_RESOURCE_PATH', '')
    _gz11 = '/usr/share/gazebo-11'
    _rp_parts = [pkg_path]
    if existing_rp:
        _rp_parts.append(existing_rp)
    if _gz11 not in existing_rp.split(':'):
        _rp_parts.append(_gz11)
    fix_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=':'.join(_rp_parts)
    )

    # ── gzserver (physics) — gui:=false, we start gzclient separately ─────
    # The default gazebo.launch.py injects libgazebo_ros_eol_gui.so which
    # causes shadow_caster timeouts and kills gzclient. Starting bare
    # gzclient ourselves avoids that entirely.
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'verbose': 'true',
            'gui':     'false',
            'world':   world_config,
        }.items(),
    )

    # ── gzclient (GUI) ─────────────────────────────────────────────────────
    gzclient = TimerAction(
        period=4.0,
        actions=[ExecuteProcess(cmd=['gzclient'], output='screen')]
    )

    # ── Robot State Publisher ──────────────────────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}],
        output='screen',
    )

    # ── Spawn robot (delayed — gzserver needs ~3s with model DB disabled) ──
    spawn = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'warden',
                    '-x', '0', '-y', '0', '-z', '0.05',
                ],
                output='screen',
            )
        ]
    )

    # ── RViz (use_sim_time=True so TF timestamps match Gazebo sim clock) ───
    rviz = TimerAction(
        period=13.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', os.path.join(pkg_path, 'rviz', 'robot.rviz')],
                parameters=[{'use_sim_time': True}],
                output='screen',
            )
        ]
    )

    overhead_cam_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='overhead_cam_tf',
        arguments=['--x', '1.0', '--y', '1.0', '--z', '3.0',
                    '--roll', '0', '--pitch', '1.5708', '--yaw', '0',
                    '--frame-id', 'odom',
                    '--child-frame-id', 'overhead_camera_link'],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        disable_online_db,   # must come before gazebo starts
        declare_world_cmd,
        fix_model_path,
        fix_resource_path,
        gzserver,
        rsp,
        overhead_cam_tf,
        gzclient,
        spawn,
        rviz,
    ])
