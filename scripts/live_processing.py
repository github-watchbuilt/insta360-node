import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from insta360_ros_driver.tools import *


class LiveProcessing(Node):
    def __init__(self):
        super().__init__("live_processing_node")
        self.bridge = CvBridge()

        self.topic_name = "/insta_image_yuv"
        self.undistort = self.declare_parameter("undistort", False).value
        self.K = np.asarray(
            self.declare_parameter("K", [[1, 0, 0], [0, 1, 0], [0, 0, 1]]).value
        )
        self.D = np.asarray(self.declare_parameter("D", [0, 0, 0, 0]).value)

        self.image_sub = self.create_subscription(
            Image, self.topic_name, self.processing, 10
        )
        self.front_image_pub = self.create_publisher(
            CompressedImage, "front_camera_image/compressed", 10
        )
        self.back_image_pub = self.create_publisher(
            CompressedImage, "back_camera_image/compressed", 10
        )

    def processing(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # Convert the YUV image to BGR format
            bgr_image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_I420)
            self.get_logger().info(
                f"Image Size: ({bgr_image.shape[1]}, {bgr_image.shape[0]})"
            )

            # Assuming the image is horizontally split for Front | Back
            height, width = bgr_image.shape[:2]
            mid_point = width // 2

            front_image = bgr_image[:, :mid_point]
            back_image = bgr_image[:, mid_point:]

            # Live Undistortion
            if self.undistort:
                front_image = undistort_image(front_image, self.K, self.D)
                back_image = undistort_image(back_image, self.K, self.D)

            # Convert to compressed image message
            front_compressed_msg = compress_image_to_msg(front_image, msg.header.stamp)
            back_compressed_msg = compress_image_to_msg(back_image, msg.header.stamp)

            # Publish the compressed images
            self.front_image_pub.publish(front_compressed_msg)
            self.back_image_pub.publish(back_compressed_msg)

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridgeError: {e}")
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")


def main(args=None):
    rclpy.init(args=args)
    live_processing = LiveProcessing()
    rclpy.spin(live_processing)
    live_processing.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
