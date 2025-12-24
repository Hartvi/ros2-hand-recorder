from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path

USE_RVIZ = "use_rviz"


def generate_launch_description():
    pkg_share = Path(__file__).resolve().parent.parent
    # TODO: add argument for urdf
    # urdf_path = pkg_share / "urdf" / "gen3_7dof" / "urdf" / "GEN3_URDF_V12.urdf"
    urdf_path = pkg_share / "urdf" / "robots" / "panda_arm.urdf"
    use_rviz = LaunchConfiguration(USE_RVIZ)
    base_link = LaunchConfiguration("base_link")
    tip_link = LaunchConfiguration("tip_link")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                USE_RVIZ,
                default_value="true",
                description="Whether to launch RViz",
            ),
            DeclareLaunchArgument(
                "robot",
                default_value="panda",
            ),
            DeclareLaunchArgument(
                "base_link",
                default_value="base_link",
            ),
            DeclareLaunchArgument(
                "tip_link",
                default_value="end_effector_link",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": urdf_path.read_text()}],
                # optional: be explicit
                # remappings=[("robot_description", "/robot_description")],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="world_to_base",
                arguments=["0", "0", "0", "0", "0", "0", "world", base_link],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(use_rviz),
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
            ),
            Node(
                package="ik_node",
                executable="trac_ik_node",
                name="trac_ik",
                output="screen",
                parameters=[
                    {"base_link": base_link},
                    {"tip_link": tip_link},
                    {"timeout": 0.02},
                    {"eps": 1e-5},
                    {"robot_description": urdf_path.read_text()},
                ],
            ),
        ]
    )
