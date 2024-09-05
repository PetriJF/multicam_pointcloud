#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String

import time
from datetime import datetime
import math
import os, yaml

class PointCloudController(Node):
    '''
    Acts as the main controller for forming the command sequence and controlling
    the multipoint cloud image acquisition software.
    '''
    def __init__(self):
        super().__init__('mcpc_controller')

        self.msg_ = String()
        self.final_sequence = ''

        self.daily_iter_ = -1
        self.timer_ = self.create_timer(60.0, self.commands)

        self.sequence_sub_ = self.create_subscription(String, 'mcpc_sequence', self.sequence_callback, 10)

        self.input_pub_ = self.create_publisher(String, 'keyboard_topic', 10)
        self.sequencer_pub_ = self.create_publisher(String, 'sequencer', 10)

        self.get_logger().info('Autonomous Controller initialized. Please generate sequence')

    def sequence_callback(self, msg: String):
        self.final_sequence = msg.data
        self.daily_iter_ = 2

        self.get_logger().info('Sequence Received.. Autonomous Controller starting')

    def commands(self):
        now = datetime.now().time()
        current_time = now.strftime('%H:%M')
        if self.daily_iter_ == -1:
            pass
        elif self.daily_iter_ == 2:
            if current_time == '07:50' or current_time == '14:00':
                self.msg_.data = 'H_0'
                self.input_pub_.publish(self.msg_)
                self.get_logger().info(f'{current_time} -- Homing the robot!')
            elif current_time == '07:40' or current_time == '13:50':
                self.msg_.data = 'C_0 Y'
                self.input_pub_.publish(self.msg_)
                self.get_logger().info(f'{current_time} -- Calibrating the Y-Axis!')
            elif current_time == '08:00' or current_time == '14:10':
                self.msg_.data = self.final_sequence
                self.sequencer_pub_.publish(self.msg_)
                self.get_logger().info(f'{current_time} -- Running the 3-Camera Imager sequence!')

       
def main(args = None):
    # MCPC Python Node installer
    rclpy.init(args = args)
    node = PointCloudController()
    
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
