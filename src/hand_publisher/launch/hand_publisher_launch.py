from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    pkg_share = Path(__file__).resolve().parent.parent
    urdf_path = pkg_share / "urdf" / "gen3_7dof" / "urdf" / "GEN3_URDF_V12.urdf"

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": urdf_path.read_text()}],
                # optional: be explicit
                remappings=[("robot_description", "/robot_description")],
            ),
            # replaced programmatically now
            # Node(
            #     package="joint_state_publisher",
            #     executable="joint_state_publisher",
            #     remappings=[("robot_description", "/robot_description")],
            # ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="world_to_base",
                arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
            ),
            # RViz2
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                # arguments=[
                #     "--ros-args",
                #     "--log-level",
                #     "rviz2:=debug",
                #     "--log-level",
                #     "rviz_rendering:=debug",
                #     "--log-level",
                #     "resource_retriever:=debug",
                # ],
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
            # --- IK node (C++) ---
            # Node(
            #     package="ik_node",
            #     executable="trac_ik_node",
            #     name="trac_ik",
            #     output="screen",
            #     parameters=[
            #         {"base_link": "base_link"},
            #         {"tip_link": "end_effector_link"},
            #         {"timeout": 0.02},
            #         {"eps": 1e-5},
            #     ],
            # ),
        ]
    )
