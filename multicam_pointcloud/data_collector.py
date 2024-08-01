#!/usr/bin/env python3
import cv2
import numpy as np
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from farmbot_interfaces.msg import ImageMessage
from sensor_msgs.msg import Image
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from datetime import datetime

class CamDataCollector(Node):
    def __init__(self, node_name, use_rz_inv):
        super().__init__(node_name)
        self.use_rz_inv = use_rz_inv

        # Determine the cameras being used
        self.declare_parameter(name = 'mcpc_camera', value = '')
        self.camera_used_:str = self.get_parameter('mcpc_camera').get_parameter_value().string_value
        accepted_cameras = ['Luxonis OAK-D Lite', 'Intel Realsense D405']
        if self.camera_used_ not in accepted_cameras:
            self.get_logger().error('CAMERA NOT SUPPORTED. Shutting down node')
            self.destroy_node()
            rclpy.shutdown()

        # Select camera config file
        self.declare_parameter(name = 'camera_config_file', value = '')
        camera_config_file:str = self.get_parameter('camera_config_file').get_parameter_value().string_value
        if camera_config_file == '':
            self.get_logger().error('No camera configuration was selected. Shutting down node')
            self.destroy_node()
            rclpy.shutdown()

        # Load Configuration
        self.config_directory = os.path.join(get_package_share_directory('multicam_pointcloud'), 'config')
        self.cam_config_data = self.load_from_yaml(self.config_directory, camera_config_file)
        system_config_file = 'mcpc_system_config.yaml'
        system_config_data = self.load_from_yaml(self.config_directory, system_config_file)
        # Robot EE position
        self.cur_x_ = 0.0
        self.cur_y_ = 0.0
        self.cur_z_ = 0.0

        # Servo position
        self.servo_0_pos = system_config_data['servo_left_rot']
        self.servo_curr_pos = 0.0

        self.bridge = CvBridge()
        self.rgb_images = {}
        self.depth_images = {}
        
        self.get_logger().info(str(self.cam_config_data))

        # Create subscriptions for image topics
        self.cam_ids = self.cam_config_data['camera_ids']
        for cam_id in self.cam_ids:
            image_topic_type = Image if self.camera_used_ == 'Intel Realsense D405' else ImageMessage
            self.create_subscription(image_topic_type, f'rgb_{cam_id}', self.rgb_callback_factory(cam_id), 10)
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
                self.servo_curr_pos = float(data_between_stars[2][1:])

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
        return self.cam_config_data[f'camera_{cam_id}']
    
    # def rgb_callback_factory(self, cam_id):
    #     def rgb_callback(msg):
    #         self.rgb_images[cam_id] = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    #         self.save_image(cam_id, 'rgb')
    #     return rgb_callback

    def rgb_callback_factory(self, cam_id):
        def rgb_callback(msg):
            if isinstance(msg, Image):
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                self.rgb_images[cam_id] = cv_image
                self.save_image(cam_id, 'rgb')
            elif isinstance(msg, ImageMessage):
                cv_image = self.bridge.imgmsg_to_cv2(msg.image, "bgr8")
                self.rgb_images[cam_id] = cv_image
                self.save_image(cam_id, 'rgb', focus = msg.focus)
            else:
                self.get_logger().warn(f"Unsupported message type for camera {cam_id}")
                return
            
            
        return rgb_callback
    
    def depth_callback_factory(self, cam_id):
        def depth_callback(msg):
            self.depth_images[cam_id] = self.bridge.imgmsg_to_cv2(msg, "mono8")
            self.save_image(cam_id, 'depth')
        return depth_callback
    
    def save_image(self, cam_id, img_type, focus: int = -1):
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
        rz = 0 if self.servo_curr_pos == self.servo_0_pos else 180
        
        # Timestamp
        now = datetime.now()
        timestamp = now.strftime("%d_%m_%Y_%H_%M_%S")
        
        # File type
        file_type = 'png' if self.camera_used_ == 'Intel Realsense D405' else 'jpeg'

        # Focus distance
        focus_dist = '' if focus == -1 else f'_{focus}'
        
        # Filename
        filename = f"{img_type}_cam{cam_id}_{timestamp}_X{x:.1f}_Y{y:.1f}_Z{z:.1f}_RX{rx:.1f}_RY{ry:.1f}_RZ{rz:.1f}{focus_dist}.{file_type}"
        
        # Directory
        directory = os.path.join(self.config_directory, 'images')
        os.makedirs(directory, exist_ok=True)
        
        # Save image
        cv2.imwrite(os.path.join(directory, filename), image)
        self.get_logger().info(f'Saved {img_type} image from camera {cam_id} as {filename}')