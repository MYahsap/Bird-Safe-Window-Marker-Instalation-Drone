"""Bridge the X650 Gazebo RGB camera to standard ROS 2 image topics."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image as RosImage

from gz.msgs10.image_pb2 import Image as GazeboImage
from gz.msgs10.image_pb2 import PixelFormatType
from gz.transport13 import Node as GazeboNode


class CameraBridge(Node):
    def __init__(self) -> None:
        super().__init__("x650_camera_bridge")
        self.declare_parameter("gz_topic", "/x650/camera/gz_image")
        self.declare_parameter("image_topic", "/x650/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/x650/camera/camera_info")
        self.declare_parameter("frame_id", "x650_camera_optical_frame")
        self.declare_parameter("horizontal_fov", math.pi / 2.0)

        self.frame_id = str(self.get_parameter("frame_id").value)
        image_topic = str(self.get_parameter("image_topic").value)
        info_topic = str(self.get_parameter("camera_info_topic").value)
        gz_topic = str(self.get_parameter("gz_topic").value)
        self.horizontal_fov = float(self.get_parameter("horizontal_fov").value)

        self.image_pub = self.create_publisher(
            RosImage, image_topic, qos_profile_sensor_data
        )
        self.info_pub = self.create_publisher(
            CameraInfo, info_topic, qos_profile_sensor_data
        )
        self.gz_node = GazeboNode()
        self.gz_node.subscribe(GazeboImage, gz_topic, self.on_gazebo_image)
        self.get_logger().info(
            f"Bridging {gz_topic} -> {image_topic} and {info_topic}"
        )

    def on_gazebo_image(self, source: GazeboImage) -> None:
        if source.pixel_format_type != PixelFormatType.RGB_INT8:
            self.get_logger().error(
                f"Unsupported Gazebo pixel format {source.pixel_format_type}; "
                "expected RGB_INT8",
                throttle_duration_sec=5.0,
            )
            return

        stamp = self.get_clock().now().to_msg()
        image = RosImage()
        image.header.stamp = stamp
        image.header.frame_id = self.frame_id
        image.height = source.height
        image.width = source.width
        image.encoding = "rgb8"
        image.is_bigendian = 0
        image.step = source.step or source.width * 3
        image.data = source.data
        self.image_pub.publish(image)

        info = CameraInfo()
        info.header = image.header
        info.height = source.height
        info.width = source.width
        fx = source.width / (2.0 * math.tan(self.horizontal_fov / 2.0))
        fy = fx
        cx = (source.width - 1.0) / 2.0
        cy = (source.height - 1.0) / 2.0
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
