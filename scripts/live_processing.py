#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
import numpy as np
from tqdm import tqdm
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

def split_image(image):
    height, width = image.shape[:2]
    mid_point = width // 2
    front_image = image[:, :mid_point]
    back_image = image[:, mid_point:]
    return front_image, back_image

def undistort_image(image, K, D):
    h, w = image.shape[:2]
    new_K = K.copy()
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), new_K, (w, h), cv2.CV_32FC1)
    undistorted_img = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR)
    return undistorted_img

def compress_image_to_msg(image, timestamp):
    _, buffer = cv2.imencode('.jpg', image)
    image_msg = CompressedImage()
    image_msg.header.stamp = timestamp
    image_msg.format = 'jpeg'
    image_msg.data = buffer.tobytes()
    return image_msg

class LiveProcessing(Node):
    def __init__(self):
        super().__init__('live_processing_node')
        self.bridge = CvBridge()

        self.topic_name = '/insta_image_yuv'
        self.undistort = self.declare_parameter('undistort', False).value
        K_flat = self.declare_parameter('K', [1, 0, 0, 0, 1, 0, 0, 0, 1]).value
        self.K = np.array(K_flat).reshape((3, 3))
        self.D = np.asarray(self.declare_parameter('D', [0, 0, 0, 0]).value)

        self.image_sub = self.create_subscription(Image, self.topic_name, self.processing, 10)
        self.front_image_pub = self.create_publisher(CompressedImage, 'front_camera_image/compressed', 10)
        self.back_image_pub = self.create_publisher(CompressedImage, 'back_camera_image/compressed', 10)
        self.front_camera_info_pub = self.create_publisher(CameraInfo, 'front_camera_info', 10)
        self.back_camera_info_pub = self.create_publisher(CameraInfo, 'back_camera_info', 10)

        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.broadcast_static_transforms()

        self.image_counter = 0  # Counter to name saved images

    def broadcast_static_transforms(self):
        # Front camera transform
        front_transform = TransformStamped()
        front_transform.header.stamp = self.get_clock().now().to_msg()
        front_transform.header.frame_id = "base_link"
        front_transform.child_frame_id = "front_frame"
        front_transform.transform.translation.x = 0.0
        front_transform.transform.translation.y = 0.0
        front_transform.transform.translation.z = 0.0
        front_transform.transform.rotation.x = 0.0
        front_transform.transform.rotation.y = 0.0
        front_transform.transform.rotation.z = 0.0
        front_transform.transform.rotation.w = 1.0

        # Back camera transform (180 degrees rotated from the front camera)
        back_transform = TransformStamped()
        back_transform.header.stamp = self.get_clock().now().to_msg()
        back_transform.header.frame_id = "base_link"
        back_transform.child_frame_id = "back_frame"
        back_transform.transform.translation.x = 0.0
        back_transform.transform.translation.y = 0.0
        back_transform.transform.translation.z = 0.0
        back_transform.transform.rotation.x = 0.0
        back_transform.transform.rotation.y = 0.0
        back_transform.transform.rotation.z = 1.0  # 180 degrees around Z-axis
        back_transform.transform.rotation.w = 0.0

        self.tf_broadcaster.sendTransform(front_transform)
        self.tf_broadcaster.sendTransform(back_transform)

    def generate_camera_info(self, width, height, K, D):
        camera_info = CameraInfo()
        camera_info.width = width
        camera_info.height = height
        camera_info.k = [float(x) for x in K.flatten()]  # Ensure all elements are floats
        camera_info.d = [float(x) for x in D]            # Ensure distortion coefficients are floats
        camera_info.r = [float(x) for x in np.eye(3).flatten()]
        camera_info.p = [float(K[0, 0]), 0.0, float(K[0, 2]), 0.0,
                        0.0, float(K[1, 1]), float(K[1, 2]), 0.0,
                        0.0, 0.0, 1.0, 0.0]
        camera_info.distortion_model = 'plumb_bob'
        return camera_info

    def processing(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Convert the YUV image to BGR format
            bgr_image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_I420)
            self.get_logger().info(f"Image Size: ({bgr_image.shape[1]}, {bgr_image.shape[0]})")

            # Assuming the image is horizontally split for Front | Back
            height, width = bgr_image.shape[:2]
            mid_point = width // 2

            front_image = bgr_image[:, :mid_point]
            back_image = bgr_image[:, mid_point:]

            # Live Undistortion
            if self.undistort:
                front_image = undistort_image(front_image, self.K, self.D)
                back_image = undistort_image(back_image, self.K, self.D)

            # Publish CameraInfo
            front_camera_info = self.generate_camera_info(front_image.shape[1], front_image.shape[0], self.K, self.D)
            back_camera_info = self.generate_camera_info(back_image.shape[1], back_image.shape[0], self.K, self.D)
            front_camera_info.header.stamp = msg.header.stamp
            front_camera_info.header.frame_id = "front_frame"
            back_camera_info.header.stamp = msg.header.stamp
            back_camera_info.header.frame_id = "back_frame"

            self.front_camera_info_pub.publish(front_camera_info)
            self.back_camera_info_pub.publish(back_camera_info)
            self.get_logger().info(f"Published CameraInfo for front and back cameras.")

            # Convert to compressed image message
            front_compressed_msg = compress_image_to_msg(front_image, msg.header.stamp)
            back_compressed_msg = compress_image_to_msg(back_image, msg.header.stamp)

            front_compressed_msg.header.frame_id = "front_frame"
            back_compressed_msg.header.frame_id = "back_frame"

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

if __name__ == '__main__':
    main()
