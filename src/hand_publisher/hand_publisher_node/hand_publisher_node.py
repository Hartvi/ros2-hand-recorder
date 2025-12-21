import numpy as np

from . import config
import rclpy
from rclpy.node import Node
from hand_publisher_interfaces.msg import HandPoints


from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation as R


class HandPublisherNode(Node):

    def __init__(
        self,
        node_name: str = "hand_publisher_node",
        topic: str = "hand_points",
        base_frame: str = "world",
    ):
        super().__init__(node_name=node_name)
        self.subscription = self.create_subscription(
            msg_type=HandPoints,
            topic=topic,
            callback=self.listener_callback,
            qos_profile=10,
        )
        self.subscription  # prevent unused variable warning

        # Publisher for RViz markers
        self.marker_pub = self.create_publisher(Marker, "hand_points_marker", 10)
        self.corrected_point_pub = self.create_publisher(
            HandPoints, "hand_points_corrected", 10
        )
        self.base_frame = base_frame

        self.old_points = np.zeros((21, 3))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def listener_callback(self, msg: HandPoints):
        hand_points = np.array(msg.points).reshape(21, 3, copy=False)
        dist = self.mix_in_distance(hand_points)
        self.normalize_hand(hand_points, dist)
        # self.get_logger().info('Hand points: "%s"' % str(dist))

        # Publish visualization marker in base_frame
        low_pass_points = self.lerp(0.2, hand_points, self.old_points)
        # self.get_logger().info(
        #     "thumb & index dist: %f"
        #     % np.linalg.norm(low_pass_points[8] - low_pass_points[4])
        # )
        hand_point_msg = HandPoints()
        hand_point_msg.points = low_pass_points.flat
        self.corrected_point_pub.publish(hand_point_msg)
        self.publish_marker(low_pass_points)
        self.old_points = hand_points

    @staticmethod
    def normalize_hand(
        points_21_3: np.ndarray,
        dist: float,
    ):
        points_21_3[:, :2] -= 0.5
        points_21_3[:, :2] *= dist
        points_21_3[:, 2] += dist

    @staticmethod
    def mix_in_distance(xyz: np.ndarray, scale=10.0):
        assert xyz.shape[0] == 21, f"{xyz=}"

        def segment_dist(x: np.ndarray):
            return np.mean(np.sqrt(np.sum(np.diff(x, axis=0) ** 2, axis=1)))

        thumb_dists = segment_dist(xyz[1:5, 0:2])
        index_dists = segment_dist(xyz[5:9, 0:2])
        middle_dists = segment_dist(xyz[9:13, 0:2])
        ring_dists = segment_dist(xyz[13:17, 0:2])
        pinky_dists = segment_dist(xyz[17:21, 0:2])
        return (
            config.TOTAL_SCALE
            * (
                config.MAX_HAND_SIZE
                / (
                    scale
                    * max(
                        thumb_dists, index_dists, middle_dists, ring_dists, pinky_dists
                    )
                )
            )
            ** config.DIST_EXPONENT
        )

    @staticmethod
    def lerp(lerp: float, arr1: np.ndarray, arr2: np.ndarray) -> np.ndarray:
        if lerp > 0.0:
            return lerp * arr1 + (1.0 - lerp) * arr2
        return arr2

    def lookup_transform(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame="world",
                source_frame="camera_frame",
                time=rclpy.time.Time(),
            )

            return transform

        except TransformException as ex:
            self.get_logger().warn(f"TF lookup failed: {ex}")

    def publish_marker(self, hand_points: np.ndarray):
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "hand_points"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST  # or Marker.POINTS
        marker.action = Marker.ADD

        # Marker pose (identity – points are already in this frame)
        marker.pose.orientation.w = 1.0

        # Size of each point
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02

        # Color (opaque green)
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        transform = self.lookup_transform()
        if transform:
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            self.get_logger().info('translation: "%s"' % str(translation))
            self.get_logger().info('rotation: "%s"' % str(rotation))
            rot = R.from_quat(
                [rotation.w, rotation.x, rotation.y, rotation.z]
            ).as_matrix()
            # Add all 21 points
            hand_points = hand_points @ rot + np.array(
                [translation.x, translation.y, translation.z]
            )
            for p in hand_points:
                pt = Point()
                pt.x = float(p[0])
                pt.y = float(p[1])
                pt.z = float(p[2])
                marker.points.append(pt)
            self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    config.init_caps()
    hand_publisher_node = HandPublisherNode(node_name="hand_publisher_node")
    rclpy.spin(hand_publisher_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
