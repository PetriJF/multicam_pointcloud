#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super.__init__('runner_node')
        self.get_logger().info('Test')