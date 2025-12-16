import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from hand_publisher_interfaces.msg import HandPoints
from .hand_utils import hand_to_pose


class HandFrameNode(Node):

    def __init__(
        self,
        node_name: str = "hand_frame_node",
        topic: str = "hand_points_corrected",
        base_frame: str = "world",
    ):
        super().__init__(node_name=node_name)
        self.br = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            msg_type=HandPoints,
            topic=topic,
            callback=self.listener_callback,
            qos_profile=10,
        )
        self.subscription  # prevent unused variable warning
        self.base_frame = base_frame

    def listener_callback(self, msg: HandPoints):
        hand_points = np.array(msg.points).reshape(21, 3, copy=False)

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.child_frame_id = "hand_frame"
        R, t = hand_to_pose(hand_points)
        qw, qx, qy, qz = list(map(float, R.as_quat()))
        tx, ty, tz = list(map(float, t))

        msg.transform.translation.x = tx
        msg.transform.translation.y = ty
        msg.transform.translation.z = tz

        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw

        self.get_logger().info('Hand points: "%s"' % str(msg))
        self.br.sendTransform(msg)


def main():
    rclpy.init()
    node = HandFrameNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
