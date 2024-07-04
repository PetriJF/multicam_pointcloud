#!/usr/bin/env python3
import rclpy
from multicam_pointcloud.data_collector import CamDataCollector

def main(args=None):
    rclpy.init(args=args)
    use_rz_inv = False  # Set this based on your input
    camera_subscriber = CamDataCollector('data_collector', use_rz_inv)
    
    try:
        rclpy.spin(camera_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        camera_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()