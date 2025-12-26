from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path

USE_RVIZ = "use_rviz"
ROBOT = "robot"


def generate_launch_description():
    pkg_share = Path(__file__).resolve().parent.parent

    ROBOT_MAPPINGS = {
        "panda": {
            "urdf_path": pkg_share / "urdf" / "robots" / "panda_arm.urdf",
            "base_link": "panda_link0",
            "tip_link": "panda_link8",
        },
        "kinova": {
            "urdf_path": pkg_share
            / "urdf"
            / "gen3_7dof"
            / "urdf"
            / "GEN3_URDF_V12.urdf",
            "base_link": "base_link",
            "tip_link": "end_effector_link",
        },
    }

    def launch_setup(context, *args, **kwargs):
        robot_name = LaunchConfiguration(ROBOT).perform(context)
        use_rviz_val = LaunchConfiguration(USE_RVIZ).perform(context).lower()

        if robot_name not in ROBOT_MAPPINGS:
            raise RuntimeError(
                f"Unknown robot='{robot_name}'. Choose one of: {list(ROBOT_MAPPINGS.keys())}"
            )

        cfg = ROBOT_MAPPINGS[robot_name]

        urdf_path = Path(cfg["urdf_path"])
        if not urdf_path.exists():
            raise RuntimeError(f"URDF path does not exist: {urdf_path}")

        urdf_xml = urdf_path.read_text()

        nodes = [
            # Robot description -> TF tree
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": urdf_xml}],
            ),
            # world -> base_link (static)
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="world_to_base",
                arguments=["0", "0", "0", "0", "0", "0", "world", cfg["base_link"]],
            ),
            # IK node (TRAC-IK)
            Node(
                package="ik_node",
                executable="trac_ik_node",
                name="trac_ik",
                output="screen",
                parameters=[
                    {
                        "base_link": cfg["base_link"],
                        "tip_link": cfg["tip_link"],
                        "timeout": 0.02,
                        "eps": 1e-5,
                        "robot_description": urdf_xml,
                    }
                ],
            ),
            Node(
                package="hand_publisher",
                namespace="hand_publisher",
                executable="hand_image_node",
                name="hand_image",
                prefix="/home/hartvi/miniconda3/bin/python",
            ),
            Node(
                package="hand_publisher",
                namespace="hand_publisher",
                executable="hand_points_node",
                name="hand_points",
                prefix="/home/hartvi/miniconda3/bin/python",
            ),
            Node(
                package="hand_publisher",
                namespace="hand_publisher",
                executable="hand_publisher_node",
                name="hand_marker",
                prefix="/home/hartvi/miniconda3/bin/python",
            ),
            Node(
                package="hand_publisher",
                namespace="hand_publisher",
                executable="hand_frame_node",
                name="hand_frame",
                prefix="/home/hartvi/miniconda3/bin/python",
            ),
            Node(
                package="hand_publisher",
                namespace="hand_publisher",
                executable="controller_node",
                name="controller",
                prefix="/home/hartvi/miniconda3/bin/python",
                parameters=[
                    {
                        "base_link": cfg["base_link"],
                    }
                ],
            ),
        ]

        # RViz (conditional)
        if use_rviz_val in ("true", "1", "yes", "on"):
            nodes.append(
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    output="screen",
                )
            )

        return nodes

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                USE_RVIZ, default_value="true", description="Whether to launch RViz"
            ),
            DeclareLaunchArgument(
                ROBOT, default_value="panda", description="Robot config key"
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
