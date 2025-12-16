import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = [f"Actuator{i}" for i in range(1, 8)]


class ControllerNode(Node):
    def __init__(self):
        super().__init__("joint_state_wave")
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.t = 0.0
        self.timer = self.create_timer(0.02, self.tick)  # 50 Hz

    def tick(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS
        msg.position = [0.4 * math.sin(self.t + i * 0.3) for i in range(len(JOINTS))]
        self.pub.publish(msg)
        self.t += 0.02


def main(args=None):
    rclpy.init()
    rclpy.spin(ControllerNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
