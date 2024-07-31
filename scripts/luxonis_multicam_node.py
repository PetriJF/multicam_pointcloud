#!/usr/bin/env python3

import cv2
import depthai as dai
import os
import time
import contextlib
from pathlib import Path
from rclpy.node import Node
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from farmbot_interfaces.msg import ImageMessage

# Manual focus set step
LENS_STEPS = [60, 70, 80, 90, 100]
FPS = 1
CAPTURE_DELAY = 1

class LuxonisMulticamNode(Node):
    '''
    Camera module that reads the Luxonis camera packets and publishes the RGB Depth stacks and depth images
    '''
    def __init__(self):
        '''
        Module constructor extending the node instance
        '''
        super().__init__('LuxonisMulticam')

        self.bridge = CvBridge()  # Bridge to convert between ROS and OpenCV images
        self.controlQueues = []
        self.stillQueues = []
        self.devices = []
        self.exit_stack = contextlib.ExitStack()  # Create an ExitStack as an attribute

        self.setup_camera()  # Initialize and configure the DepthAI camera
        self.image_message_ = ImageMessage()
        # Initialize publishers for RGB and depth images
        self.image_publisher = self.create_publisher(ImageMessage, 'rgb_img', 10)
        self.depth_publisher = self.create_publisher(Image, 'depth_img', 10)

        self.rgb_image_ = None
        self.depth_image_ = None

        # Capture depth stacks once for testing
        self.capture_depth_stacks()

        self.get_logger().info('Luxonis Multicam Node initialized...')
    
    def createPipeline(self, idx):
        pipeline = dai.Pipeline()
        camRgb = pipeline.create(dai.node.ColorCamera)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setFps(FPS)

        controlIn = pipeline.create(dai.node.XLinkIn)
        stillOut = pipeline.create(dai.node.XLinkOut)
        videoEnc = pipeline.create(dai.node.VideoEncoder)
        videoEnc.setDefaultProfilePreset(1, dai.VideoEncoderProperties.Profile.MJPEG)
        
        controlIn.setStreamName(f'control-{idx}')
        stillOut.setStreamName(f'still-{idx}')

        camRgb.still.link(videoEnc.input)
        videoEnc.bitstream.link(stillOut.input)
        controlIn.out.link(camRgb.inputControl)

        return pipeline
    
    def setup_camera(self):
        '''
        Sets the camera links and pipeline
        '''
        deviceInfos = dai.Device.getAllAvailableDevices()
        if len(deviceInfos) < 1:
            print("Please connect at least 1 device.")
            exit(1)

        for idx, deviceInfo in enumerate(deviceInfos):
            try:
                usb_speed = dai.UsbSpeed.HIGH
                device = self.exit_stack.enter_context(dai.Device(deviceInfo, usb_speed))
                self.devices.append(device)
                pipeline = self.createPipeline(idx)
                device.startPipeline(pipeline)
                controlQueue = device.getInputQueue(f'control-{idx}')
                stillQueue = device.getOutputQueue(f'still-{idx}', maxSize=1, blocking=True)
                self.controlQueues.append(controlQueue)
                self.stillQueues.append(stillQueue)
                print(f"Connected to device {idx+1} with MXID: {device.getMxId()}")
            except Exception as e:
                print(f"Failed to connect to device {idx+1}, error: {e}")
                exit(1)
            
    def publish_images(self, rgb_frame=None, focus=None, id=None, depth_frame=None):
        '''
        Publish RGB and depth frames if available
        '''
        try:
            if rgb_frame is not None:
                if rgb_frame.shape[2] == 3:  # Ensure the frame has 3 channels
                    self.image_message_.image = self.bridge.cv2_to_imgmsg(rgb_frame, encoding='bgr8')
                    self.image_message_.id = id
                    self.image_message_.focus = focus
                    self.image_publisher.publish(self.image_message_)
                else:
                    self.get_logger().error(f"RGB frame for Camera {id} does not have 3 channels")
            if depth_frame is not None:
                self.image_message_.image = self.bridge.cv2_to_imgmsg(depth_frame, encoding='mono8')
                self.image_message_.id = id
                self.image_message_.focus = None
                self.image_publisher.publish(self.image_message_)
        except Exception as e:
            self.get_logger().error(f"Failed to publish images: {e}")
        
    def clamp(num, v0, v1):
        return max(v0, min(num, v1))

    def cleanup(self, devices):
        for device in devices:
            device.close()

    def capture_depth_stacks(self):
        lensPositions = [50, 50, 50]

        for controlQueue in self.controlQueues:
            try:
                print("Sending initial manual focus to control queue")
                ctrl = dai.CameraControl()
                ctrl.setManualFocus(50)
                controlQueue.send(ctrl)
            except Exception as e:
                print(f"Error setting initial manual focus: {type(e).__name__}: {e}")

        cv2.waitKey(500)

        for lens_position in LENS_STEPS:
            for i, stillQueue in enumerate(self.stillQueues):
                try:
                    print(f"Sending capture still to control queue for Camera {i+1}")
                    ctrl = dai.CameraControl()
                    ctrl.setCaptureStill(True)
                    self.controlQueues[i].send(ctrl)
                    print(f"Camera {i+1} - Sent 'still' event to the camera!")

                    stillFrame = stillQueue.get()  # Blocking call, will wait until a new data has arrived
                    if stillFrame:
                        frame_data = stillFrame.getData()
                        if frame_data is not None and frame_data.size > 0:
                            rgb_image = cv2.imdecode(frame_data, cv2.IMREAD_COLOR)
                            if rgb_image is not None:
                                self.publish_images(rgb_frame=rgb_image, id=i+1, focus=lensPositions[i])
                                lensPositions[i] = lens_position
                                print(f"Camera {i+1} - Setting manual focus, lens position: {lensPositions[i]}")
                                ctrl = dai.CameraControl()
                                ctrl.setManualFocus(lensPositions[i])
                                self.controlQueues[i].send(ctrl)
                            else:
                                print(f"Camera {i+1} - Failed to decode frame data")
                        else:
                            print(f"Camera {i+1} - Received empty frame data")

                        # Clear the stillQueue to avoid filling up the buffer
                        stillQueue.tryGetAll()

                    time.sleep(CAPTURE_DELAY)  # Delay to allow the device to process the frame

                except RuntimeError as e:
                    print(f"Camera {i+1} encountered an error while sending control command: {e}")
                    self.cleanup(self.devices)
                    exit(1)
                key = cv2.waitKey(1)
                if key == ord('q'):
                    self.cleanup(self.devices)
                    exit(0)

        print("Depth stack capture complete.")

    def destroy_node(self):
        super().destroy_node()
        self.exit_stack.close()  # Close the ExitStack when the node is destroyed

def main(args=None):
    rclpy.init(args=args)
    node = LuxonisMulticamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Luxonis Multicam Node...")
    finally:
        node.cleanup(node.devices)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
