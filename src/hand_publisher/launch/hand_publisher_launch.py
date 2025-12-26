from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Static TF: world -> base_link
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="world_to_base",
                arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
            ),
            Node(
                package="hand_publisher",
                namespace="hand_publisher",
                executable="hand_image_node",
                name="hp1",
                prefix="/home/hartvi/miniconda3/bin/python",
            ),
            Node(
                package="hand_publisher",
                namespace="hand_publisher",
                executable="hand_points_node",
                name="h",
                prefix="/home/hartvi/miniconda3/bin/python",
            ),
            Node(
                package="hand_publisher",
                namespace="hand_publisher",
                executable="hand_publisher_node",
                name="asdf",
                prefix="/home/hartvi/miniconda3/bin/python",
            ),
        ]
    )
