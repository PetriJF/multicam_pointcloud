#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from farmbot_interfaces.msg import ImageMessage
from cv_bridge import CvBridge
import os
import cv2

class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver_node')
        self.subscription = self.create_subscription(
            ImageMessage,
            'rgb_img',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.save_dirs = {
            1: 'depth_stack_images_cam1_2',
            2: 'depth_stack_images_cam2_2',
            3: 'depth_stack_images_cam3_2'
        }

        for save_dir in self.save_dirs.values():
            os.makedirs(save_dir, exist_ok=True)

        self.get_logger().info('ImageSaverNode initialized...')

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg.image, "bgr8")
            save_dir = self.save_dirs.get(msg.id, 'unknown_cam')
            if save_dir == 'unknown_cam':
                os.makedirs(save_dir, exist_ok=True)
                
            file_name = f"id_{msg.id}_focus_{msg.focus}.jpeg"
            file_path = os.path.join(save_dir, file_name)
            cv2.imwrite(file_path, cv_image)
            self.get_logger().info(f"Saved image: {file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save image: {e}")

def main(args=None):
    rclpy.init(args=args)
    image_saver_node = ImageSaverNode()
    rclpy.spin(image_saver_node)
    image_saver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
