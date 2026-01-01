#!/usr/bin/env python3
import numpy as np
import rclpy
from hand_publisher_interfaces.msg import HandPoints
from rclpy.node import Node
from sensor_msgs.msg import JointState


class GripperPublisher(Node):
    def __init__(self):
        super().__init__("gripper_publisher")
        self.pub = self.create_publisher(JointState, "/gripper_joint_states", 10)
        self.sub = self.create_subscription(
            HandPoints, "hand_points_corrected", self.grip, 10
        )
        # TODO: process handpoints into a gripper position
        self.q = 0.0  # finger_joint angle (rad)

    def grip(self, msg: HandPoints):

        hand_points = np.array(msg.points).reshape(21, 3, copy=False)
        dist = np.linalg.norm(hand_points[4] - hand_points[8])
        self.q = max(0, min(1, float(1 - 10 * dist)))
        # self.get_logger().info('Hand points: "%s"' % str(self.q))
        self.publish()

    def publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        # TODO: get this as parameters
        msg.name = [
            "finger_joint",
            "left_inner_knuckle_joint",
            "right_outer_knuckle_joint",
            "right_inner_knuckle_joint",
            "left_inner_finger_joint",
            "right_inner_finger_joint",
        ]

        msg.position = [
            self.q,  # finger_joint
            self.q,  # +1 mimic
            self.q,  # +1 mimic
            self.q,  # +1 mimic
            -self.q,  # -1 mimic
            -self.q,  # -1 mimic
        ]

        self.pub.publish(msg)

    def link_positions(self):
        link_names = ["left_inner_finger_pad", "right_inner_finger_pad"]
        """
        ee transform
        =>
        left pad
        right pad
        diff vec 1 = finger point 1 - left pad.position
        diff vec 2 = finger point 2 - right pad.position
        diff vec = 0.5 * (diff vec 1 + diff vec 2)

        PROPOSAL 2:
        ik on left finger + rotation to match the other finger
        set the rotation to match the other finger

        """


def main():
    rclpy.init()
    node = GripperPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
