#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateMerger(Node):
    def __init__(self):
        super().__init__("joint_state_merger")

        self.arm = None
        self.gripper = None

        self.sub_arm = self.create_subscription(
            JointState, "/joint_states", self.cb_arm, 10
        )
        self.sub_grip = self.create_subscription(
            JointState, "/gripper_joint_states", self.cb_grip, 10
        )

        self.pub = self.create_publisher(JointState, "/joint_states_merged", 10)
        self.timer = self.create_timer(1.0 / 30.0, self.tick)

    def cb_arm(self, msg: JointState):
        self.arm = msg

    def cb_grip(self, msg: JointState):
        self.gripper = msg

    def tick(self):
        # Need at least the arm states to do anything useful
        if self.arm is None:
            return

        merged = JointState()
        merged.header.stamp = self.get_clock().now().to_msg()

        name_to_pos = {}

        def ingest(msg: JointState | None):
            if msg is None:
                return
            # positions are what robot_state_publisher needs; ignore missing arrays safely
            for i, n in enumerate(msg.name):
                if i < len(msg.position):
                    name_to_pos[n] = msg.position[i]

        ingest(self.arm)
        ingest(self.gripper)

        merged.name = list(name_to_pos.keys())
        merged.position = [name_to_pos[n] for n in merged.name]

        self.pub.publish(merged)


def main():
    rclpy.init()
    node = JointStateMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
