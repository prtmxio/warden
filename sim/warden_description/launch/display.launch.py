import re
import sys
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_root  = Path(__file__).resolve().parent.parent
    urdf_path = pkg_root / "models" / "urdf" / "warden.urdf"
    rviz_path = pkg_root / "rviz" / "robot.rviz"

    if not urdf_path.exists():
        print(f"[ERROR] URDF not found: {urdf_path}", file=sys.stderr)
        raise SystemExit(1)

    # Auto-discover every STL anywhere in the package tree, build name→path map.
    # This is resilient to the STLs ending up in meshes/, assets/, or the root.
    stl_map = {f.name: f for f in pkg_root.rglob("*.stl")}
    if not stl_map:
        print("[ERROR] No STL files found under package root!", file=sys.stderr)
        print(f"        Searched: {pkg_root}", file=sys.stderr)
        raise SystemExit(1)
    sample_dir = next(iter(stl_map.values())).parent
    print(f"[INFO] Found {len(stl_map)} STL file(s) in: {sample_dir}")

    robot_description = urdf_path.read_text()

    def _resolve(m: re.Match) -> str:
        name = m.group(1)
        path = stl_map.get(name)
        if path is None:
            print(f"[WARN] STL not found in package: {name}", file=sys.stderr)
            return m.group(0)   # leave original so RViz gives a clear error
        return f"file://{path}"

    robot_description = re.sub(
        r'package://[^"]+/([^"/]+\.stl)',
        _resolve,
        robot_description,
    )

    patched = robot_description.count("file://")
    if patched == 0:
        print("[WARN] No mesh URIs were resolved — check URDF mesh filenames.", file=sys.stderr)
    else:
        print(f"[INFO] Resolved {patched} mesh URI(s) to absolute file:// paths.")

    rviz_args = (
        ["-d", str(rviz_path)]
        if rviz_path.exists() and rviz_path.stat().st_size > 0
        else []
    )

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": False,
            }],
            output="screen",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=rviz_args,
            output="screen",
        ),
    ])
