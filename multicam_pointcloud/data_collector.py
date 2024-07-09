#!/usr/bin/env python3
import cv2
import numpy as np
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from datetime import datetime

class CamDataCollector(Node):
    def __init__(self, node_name, use_rz_inv):
        super().__init__(node_name)
        self.use_rz_inv = use_rz_inv
        
        # Configuration
        self.config_directory = os.path.join(get_package_share_directory('multicam_pointcloud'), 'config')
        self.camera_config_file = 'rs_405_camera_config.yaml'
        self.config_data = self.load_from_yaml(self.config_directory, self.camera_config_file)
     
        # Robot EE position
        self.cur_x_ = 0.0
        self.cur_y_ = 0.0
        self.cur_z_ = 0.0

        # Servo position
        self.servo_pos = 0.0

        self.bridge = CvBridge()
        self.rgb_images = {}
        self.depth_images = {}
        
        self.get_logger().info(str(self.config_data))

        # Create subscriptions for image topics
        self.cam_ids = self.config_data['camera_ids']
        for cam_id in self.cam_ids:
            self.create_subscription(Image, f'rgb_{cam_id}', self.rgb_callback_factory(cam_id), 10)
            self.create_subscription(Image, f'depth_{cam_id}', self.depth_callback_factory(cam_id), 10)
            self.get_logger().info(f'Initializing Topics /rgb_{cam_id} and /depth_{cam_id}')
        
        # UART Rx Subscriber
        self.uart_rx_sub_ = self.create_subscription(String, 'uart_receive', self.uart_feedback_callback, 10)

        self.get_logger().info("CamDataCollector node has been initialized")

    def uart_feedback_callback(self, msg: String):
        '''
        Takes the feedback from the Serial Receiver and updates
        information accordingly 
        '''
        msgSplit = (msg.data).split(' ')
        reportCode = msgSplit[0]

        if reportCode == 'R82':
            self.cur_x_ = float(msgSplit[1][1:])
            self.cur_y_ = float(msgSplit[2][1:])
            self.cur_z_ = float(msgSplit[3][1:])
        elif reportCode == 'R08':
            data_between_stars = (' '.join(msgSplit[1:]).split('*')[1]).split(' ')
            # Check if servo command was received
            if data_between_stars[0] == 'F61':
                self.servo_pos = float(data_between_stars[2][1:])

    def load_from_yaml(self, path, file_name):
        full_path = os.path.join(path, file_name)
        if not os.path.exists(full_path):
            self.get_logger().warn(f"File path is invalid: {full_path}")
            return None
        
        with open(full_path, 'r') as yaml_file:
            try:
                return yaml.safe_load(yaml_file)
            except yaml.YAMLError as e:
                self.get_logger().warn(f"Error reading YAML file: {e}")
                return None
    
    def get_camera_config(self, cam_id):
        return self.config_data[f'camera_{cam_id}']
    
    def rgb_callback_factory(self, cam_id):
        def rgb_callback(msg):
            self.rgb_images[cam_id] = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.save_image(cam_id, 'rgb')
        return rgb_callback
    
    def depth_callback_factory(self, cam_id):
        def depth_callback(msg):
            self.depth_images[cam_id] = self.bridge.imgmsg_to_cv2(msg, "mono8")
            self.save_image(cam_id, 'depth')
        return depth_callback
    
    def save_image(self, cam_id, img_type):
        image = self.rgb_images[cam_id] if img_type == 'rgb' else self.depth_images[cam_id]
        
        # Get camera configuration
        cam_config = self.get_camera_config(cam_id)
        
        # Get robot position and offsets
        x = self.cur_x_ + cam_config['offset_x']
        y = self.cur_y_ + cam_config['offset_y']
        z = self.cur_z_ + cam_config['offset_z']
        
        # Get rotation
        rx = cam_config['rx']
        ry = cam_config['ry']
        rz = cam_config['rz'] if not self.servo_pos else cam_config['rz_inv']
        
        # Timestamp
        now = datetime.now()
        timestamp = now.strftime("%d_%m_%Y_%H_%M_%S")
        
        # Filename
        filename = f"{img_type}_cam{cam_id}_{timestamp}_{x:.1f}_{y:.1f}_{z:.1f}_X{rx:.1f}_Y{ry:.1f}_Z{rz:.1f}.png"
        
        # Directory
        directory = os.path.join(self.config_directory, 'images')
        os.makedirs(directory, exist_ok=True)
        
        # Save image
        cv2.imwrite(os.path.join(directory, filename), image)
        self.get_logger().info(f'Saved {img_type} image from camera {cam_id} as {filename}')