import cv2
import depthai as dai
import os
import yaml
import contextlib
from rclpy.node import Node
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from farmbot_interfaces.msg import ImageMessage
from farmbot_interfaces.srv import StringRepReq
from ament_index_python.packages import get_package_share_directory
from multicam_pointcloud.data_manager import CamDataManager
import numpy as np

# Manual focus set step
LENS_STEPS = []
FPS = 5

class LuxonisMulticam(Node):
    '''
    Camera module that reads the Luxonis camera packets and publishes the RGB Depth stacks and depth images
    '''
    def __init__(self):
        '''
        Module constructor extending the node instance
        '''
        super().__init__('LuxonisMulticam')

        ### Extend node to data manager module
        self.img_mngr_ = CamDataManager(self)
        
        ### Camera variables and objects
        self.bridge = CvBridge()
        self.controlQueues = []
        self.stillQueues = []
        self.devices = []
        self.leftQueues = []
        self.rightQueues = []
        self.stereoQueues = []
        self.exit_stack = contextlib.ExitStack()  # Create an ExitStack as an attribute


        self.config_directory = os.path.join(get_package_share_directory('multicam_pointcloud'), 'config')
        camera_config_file = 'luxonis_oak_d_lite_camera_config.yaml'
        self.cam_config_data = self.load_from_yaml(self.config_directory, camera_config_file)
        
        self.rgb_publishers = {}
        self.mono_publishers = {}
        self.depth_publishers = {}
        self.setup_camera()  # Initialize and configure the DepthAI camera
        self.image_message_ = ImageMessage()
        # Dictionary to hold publishers keyed by camera id

        # self.create_publishers()  # Initialize publishers for RGB, depth, and mono images

        self.rgb_image_ = None
        self.depth_image_ = None

        # Create service to take images and publish them to the required topics
        self.luxonis_image_server_ = self.create_service(StringRepReq, 'multicam_toggle', self.capture_depth_stacks_server)

        # for i in range(200):
        #     self.test_publish()
        # FOR DEBUGGING: Capture depth stacks once for testing
        # self.capture_depth_stacks()

        self.get_logger().info('Luxonis Multicam Node initialized...')

    def configure_stereo_depth(self, stereo):
        '''
        Apply stereo depth settings from configuration amd disparity post-processing
        '''
        # Apply stereo depth settings from configuration
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        stereo.setLeftRightCheck(True)
        stereo.setExtendedDisparity(False)
        stereo.setSubpixel(True)
        
        # Additional disparity post-processing
        config = stereo.initialConfig.get()
        config.postProcessing.temporalFilter.enable = True
        config.postProcessing.spatialFilter.enable = True
        stereo.initialConfig.set(config)
        
    def create_pipeline(self, idx):
        pipeline = dai.Pipeline()
        camRgb = pipeline.create(dai.node.ColorCamera)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_4_K)
        camRgb.setFps(FPS)

        monoRight = pipeline.create(dai.node.MonoCamera)
        monoLeft = pipeline.create(dai.node.MonoCamera)
        monoRight.setCamera('right')
        monoLeft.setCamera('left')
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)

        stereo = pipeline.create(dai.node.StereoDepth)

        controlIn = pipeline.create(dai.node.XLinkIn)
        stillOut = pipeline.create(dai.node.XLinkOut)
        videoEnc = pipeline.create(dai.node.VideoEncoder)
        videoEnc.setDefaultProfilePreset(1, dai.VideoEncoderProperties.Profile.MJPEG)
        xoutLeft = pipeline.create(dai.node.XLinkOut)
        xoutRight = pipeline.create(dai.node.XLinkOut)
        xoutStereo = pipeline.create(dai.node.XLinkOut)

        controlIn.setStreamName(f'control-{idx}')
        stillOut.setStreamName(f'still-{idx}')
        xoutLeft.setStreamName(f'left-{idx}')
        xoutRight.setStreamName(f'right-{idx}')
        xoutStereo.setStreamName(f'stereo-{idx}')

        self.configure_stereo_depth(stereo)

        camRgb.still.link(videoEnc.input)
        videoEnc.bitstream.link(stillOut.input)
        controlIn.out.link(camRgb.inputControl)
        monoRight.out.link(xoutRight.input)
        monoLeft.out.link(xoutLeft.input)
        monoRight.out.link(stereo.right)
        monoLeft.out.link(stereo.left)
        stereo.disparity.link(xoutStereo.input)
        return pipeline

    def setup_camera(self):
        '''
        Sets the camera links and pipeline
        '''
        deviceInfos = dai.Device.getAllAvailableDevices()
        if len(deviceInfos) < 1:
            self.get_logger().info('Please connect at least 1 device.')
            exit(1)

        for idx, deviceInfo in enumerate(deviceInfos):
            try:
                usb_speed = dai.UsbSpeed.HIGH
                device = self.exit_stack.enter_context(dai.Device(deviceInfo, usb_speed))
                self.devices.append(device)
                pipeline = self.create_pipeline(idx)
                device.startPipeline(pipeline)
                controlQueue = device.getInputQueue(f'control-{idx}')
                stillQueue = device.getOutputQueue(f'still-{idx}', maxSize=1, blocking=False)
                leftQueue = device.getOutputQueue(f'left-{idx}', maxSize=1, blocking=False)
                rightQueue = device.getOutputQueue(f'right-{idx}', maxSize=1, blocking=False)
                stereoQueue = device.getOutputQueue(f'stereo-{idx}', maxSize=1, blocking=False)
                self.controlQueues.append(controlQueue)
                self.stillQueues.append(stillQueue)
                self.leftQueues.append(leftQueue)
                self.rightQueues.append(rightQueue)
                self.stereoQueues.append(stereoQueue)
                self.get_logger().info(f'Connected to device {idx+1} with MXID: {device.getMxId()}')
            except Exception as e:
                self.get_logger().info(f'Failed to connect to device {idx+1}, error: {e}')
                exit(1)
        
    def clamp(num, v0, v1):
        return max(v0, min(num, v1))

    def capture_depth_stacks_server(self, request, response):
        if request.data == 'TAKE':
            if 1 > 2:   # TODO add checks here
                response.data = 'FAILED'
                return response
            
            result = self.capture_depth_stacks()
            response.data = 'SUCCESS' if result else 'FAILED'
            return response

    def capture_depth_stacks(self):
        for controlQueue in self.controlQueues:
            try:
                self.get_logger().info('Sending initial manual focus to control queue')
                ctrl = dai.CameraControl()
                ctrl.setManualFocus(150)
                controlQueue.send(ctrl)
            except Exception as e:
                self.get_logger().info(f'Error setting initial manual focus: {type(e).__name__}: {e}')

        cv2.waitKey(500)
        lens_steps_a = self.cam_config_data[f'camera_{self.devices[0].getMxId()}']['lens_steps']
        for j in range(len(lens_steps_a)):
            for i, stillQueue in enumerate(self.stillQueues):
                cam_id = self.devices[i].getMxId()
                lens_position = self.cam_config_data[f'camera_{cam_id}']['lens_steps'][j]
                try:
                    ctrl = dai.CameraControl()
                    ctrl.setCaptureStill(True)
                    self.controlQueues[i].send(ctrl)
                    self.get_logger().info(f"Camera {i+1} - Sent 'still' event to the camera!")

                    stillFrame = stillQueue.get()  # Blocking call, will wait until a new data has arrived
                    if stillFrame:
                        frame_data = stillFrame.getData()
                        if frame_data is not None and frame_data.size > 0:
                            rgb_image = cv2.imdecode(frame_data, cv2.IMREAD_COLOR)
                            if rgb_image is not None:
                                # Downscale the image by a factor of 10
                                #height, width = rgb_image.shape[:2]
                                #new_dimensions = (width // 2, height // 2)
                                #rgb_image = cv2.resize(rgb_image, new_dimensions, interpolation=cv2.INTER_AREA)
                                self.img_mngr_.save_img('rgb', cam_id, rgb_image, lens_position)
                                self.get_logger().info(f'Camera {i+1} - Setting manual focus, lens position: {lens_position}')
                                ctrl = dai.CameraControl()
                                ctrl.setManualFocus(lens_position)
                                self.controlQueues[i].send(ctrl)
                            else:
                                self.get_logger().info(f'Camera {i+1} - Failed to decode frame data')
                        else:
                            self.get_logger().info(f'Camera {i+1} - Received empty frame data')

                        # Clear the stillQueue to avoid filling up the buffer
                        stillQueue.tryGetAll()

                except RuntimeError as e:
                    self.get_logger().info(f'Camera {i+1} encountered an error while sending control command: {e}')
                    self.cleanup(self.devices)
                    self.destroy_node()
                    return False 

        # do stereo depth capture here
        for i, leftQueue in enumerate(self.leftQueues):
            cam_id = self.devices[i].getMxId()
            rightQueue = self.rightQueues[i]
            leftFrame = leftQueue.get()
            rightFrame = rightQueue.get()
            self.img_mngr_.save_img('mono', cam_id, leftFrame.getCvFrame(), focus=-10)
            self.img_mngr_.save_img('mono', cam_id, rightFrame.getCvFrame(), focus=-11)
        for i, stereoQueue in enumerate(self.stereoQueues):
            cam_id = self.devices[i].getMxId()
            stereoFrame = stereoQueue.get()
            self.img_mngr_.save_img('depth', cam_id, stereoFrame.getCvFrame(), focus=-9)
        
        self.get_logger().info('Depth stack capture complete.')
        return True  

    def load_from_yaml(self, path, file_name):
        '''
        Returns a yaml file from the selected path and file name
        '''
        full_path = os.path.join(path, file_name)
        if not os.path.exists(full_path):
            self.get_logger().warn(f'File path is invalid: {full_path}')
            return None
        
        with open(full_path, 'r') as yaml_file:
            try:
                return yaml.safe_load(yaml_file)
            except yaml.YAMLError as e:
                self.get_logger().warn(f'Error reading YAML file: {e}')
                return None

    def cleanup(self, devices):
        for device in devices:
            device.close()

    def destroy_node(self):
        super().destroy_node()
        self.exit_stack.close()  # Close the ExitStack when the node is destroyed

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
