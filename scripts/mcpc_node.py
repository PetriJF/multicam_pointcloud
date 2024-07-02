#!/usr/bin/env python3

import rclpy
from multicam_pointcloud.mcpc_controller import PointCloudController

def main(args = None):
    # MCPC Python Node installer
    rclpy.init(args = args)
    node = PointCloudController()
     
    try:
        while rclpy.ok():
            node.controller()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()