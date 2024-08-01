#!/usr/bin/env python3
import rclpy
from multicam_pointcloud.luxonis_multicam import LuxonisMulticam

def main(args=None):
    rclpy.init(args=args)
    node = LuxonisMulticam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Luxonis Multicam Node...')
    finally:
        node.cleanup(node.devices)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
