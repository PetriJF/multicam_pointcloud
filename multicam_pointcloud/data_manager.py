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
 
class CamDataManager():
    def __init__(self, node:  Node):
        self.node_ = node
        
        # Determine the cameras being used
        self.node_.declare_parameter(name='mcpc_camera', value='')
        self.camera_used_: str = self.node_.get_parameter('mcpc_camera').get_parameter_value().string_value
        accepted_cameras = ['Luxonis OAK-D Lite', 'Intel Realsense D405']
        if self.camera_used_ not in accepted_cameras:
            self.node_.get_logger().error('CAMERA NOT SUPPORTED. Shutting down node')
            self.node_.destroy_node()
            rclpy.shutdown()
 
        # Select camera config file
        self.node_.declare_parameter(name='camera_config_file', value='')
        camera_config_file: str = self.node_.get_parameter('camera_config_file').get_parameter_value().string_value
        if camera_config_file == '':
            self.node_.get_logger().error('No camera configuration was selected. Shutting down node')
            self.node_.destroy_node()
            rclpy.shutdown()
 
        # Load Configuration
        self.config_directory = os.path.join(get_package_share_directory('multicam_pointcloud'), 'config')
        self.cam_config_data = self.load_from_yaml(self.config_directory, camera_config_file)
        system_config_file = 'mcpc_system_config.yaml'
        system_config_data = self.load_from_yaml(self.config_directory, system_config_file)
 
        self.node_.declare_parameter(name='image_save_path', value='')
        self.image_save_dir_: str = self.node_.get_parameter('image_save_path').get_parameter_value().string_value
        if self.image_save_dir_ == '':
            self.node_.get_logger().error('Save directory was not selected in the launch file. Shutting down node')
            self.node_.destroy_node()
            rclpy.shutdown()
 
        self.node_.declare_parameter(name='daily_measurement_count', value=1)
        self.daily_measurement_count_: int = self.node_.get_parameter('daily_measurement_count').get_parameter_value().integer_value
 
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
        self.mono_images = {}
 
        self.node_.get_logger().info(str(self.cam_config_data))
 
        # Create subscriptions for image topics
        self.cam_ids = self.cam_config_data['camera_ids']
        self.uart_rx_sub_ = self.node_.create_subscription(String, 'uart_receive', self.uart_feedback_callback, 10)
 
        self.node_.get_logger().info("CamDataManager node has been initialized")
 
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
        self.node_.get_logger().info(f'Loading YAML file from: {full_path}')
        if not os.path.exists(full_path):
            self.node_.get_logger().warn(f'File path is invalid: {full_path}')
            return None
 
        with open(full_path, 'r') as yaml_file:
            try:
                return yaml.safe_load(yaml_file)
            except yaml.YAMLError as e:
                self.node_.get_logger().warn(f'Error reading YAML file: {e}')
                return None
 
    def get_camera_config(self, cam_id):
        return self.cam_config_data[f'camera_{cam_id}']

    def save_img(self, type, cam_id, cv_image, focus = -1):
        if type == 'rgb':
            self.rgb_images[cam_id] = cv_image
            self.save_image(cam_id, 'rgb', focus)
        elif type == 'mono':
            self.mono_images[cam_id] = cv_image
            self.save_image(cam_id, 'mono', focus=focus)
        elif type == 'depth':
            self.depth_images[cam_id] = cv_image
            self.save_image(cam_id, 'depth', focus=focus)

 
    def save_image(self, cam_id, img_type, focus: int = -1):
        self.node_.get_logger().info(f'Saving {img_type} image from camera {cam_id}')
        image = self.rgb_images[cam_id] if img_type == 'rgb' else self.depth_images[cam_id] if img_type == 'depth' else self.mono_images[cam_id]
 
        # Get camera configuration
        cam_config = self.get_camera_config(cam_id)
 
        # Get rotation
        rx = cam_config['rx']
        ry = cam_config['ry']
        rz = 0 if self.servo_curr_pos == self.servo_0_pos else 180

        # Get robot position and offsets
        x = self.cur_x_ + cam_config['offset_x'] * (-1 if rz == 180 else 0)
        y = self.cur_y_ + cam_config['offset_y']
        z = self.cur_z_ + cam_config['offset_z']
 
        # Timestamp
        now = datetime.now()
        timestamp = now.strftime("%d_%m_%Y_%H_%M_%S")
 
        # File type
        file_type = 'png'
 
        # Focus distance
        focus_dist = '' if focus == -1 else '_stereo_disparity' if focus == -9 else '_mono_left' if focus == -10 else '_mono_right' if focus == -11 else f'_{focus}'
 
        # Filename
        if img_type == 'depth':
            filename = f"{img_type}_cam{cam_id}_{timestamp}_X{x:.1f}_Y{y:.1f}_Z{z:.1f}_RX{rx:.1f}_RY{ry:.1f}_RZ{rz:.1f}{focus_dist}.jpeg"
        else:
            filename = f"{img_type}_cam{cam_id}_{timestamp}_X{x:.1f}_Y{y:.1f}_Z{z:.1f}_RX{rx:.1f}_RY{ry:.1f}_RZ{rz:.1f}{focus_dist}.{file_type}"

        # Save folder
        now = datetime.now()
        date = now.strftime("%Y-%m-%d")
        folder_name = ''
        if self.daily_measurement_count_ == 0:
            folder_name = 'images'
        elif self.daily_measurement_count_ == 1:
            folder_name = date
        elif self.daily_measurement_count_ == 2:
            hour = now.hour
            if hour < 12:
                folder_name = date + '-Morning'
            else:
                folder_name = date + '-Evening'
        else:
            self.node_.get_logger().error(f'NOT IMPLEMENTED FOR {self.daily_measurement_count_} MEASUREMENTS')
 
        # Directory
        directory = os.path.join(self.image_save_dir_, folder_name)
        os.makedirs(directory, exist_ok=True)
 
        # Save image
        full_save_path = os.path.join(directory, filename)
        cv2.imwrite(full_save_path, image)
        self.node_.get_logger().info(f'Successfully saved {img_type} image from camera {cam_id} as {filename}\nLocation: {full_save_path}')
