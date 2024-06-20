// src/realsense_multicam_node.cpp

#include "multicam_pointcloud/realsense_multicam_node.hpp"

RealSenseMultiCameraNode::RealSenseMultiCameraNode() : Node("realsense_multi_camera")
{
    setup_camera();
    RCLCPP_INFO(this->get_logger(), "RealSense Multi Camera Node initialized...");

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),  // 5 FPS
        std::bind(&RealSenseMultiCameraNode::capture_and_publish, this));
}

RealSenseMultiCameraNode::~RealSenseMultiCameraNode()
{
    for (auto &pipe : pipelines_)
    {
        pipe.stop();
    }
}

void RealSenseMultiCameraNode::setup_camera()
{
    rs2::context ctx;
    std::vector<std::string> serials;
    for (auto &&dev : ctx.query_devices())
    {
        std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        serials.push_back(serial);
        RCLCPP_INFO(this->get_logger(), "Connected camera serial number: %s", serial.c_str());
    }

    for (const auto &serial : serials)
    {
        rs2::pipeline *pipe = new rs2::pipeline(ctx);
        rs2::config cfg;
        cfg.enable_device(serial);
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 5);
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 5);
        pipe->start(cfg);
        pipelines_.push_back(*pipe);
        serial_to_pipeline_[serial] = pipe;

        auto rgb_publisher = this->create_publisher<sensor_msgs::msg::Image>("rgb_" + serial, 10);
        auto depth_publisher = this->create_publisher<sensor_msgs::msg::Image>("depth_" + serial, 10);
        rgb_publishers_[serial] = rgb_publisher;
        depth_publishers_[serial] = depth_publisher;
    }
}

void RealSenseMultiCameraNode::capture_and_publish()
{
    for (const auto &pair : serial_to_pipeline_)
    {
        const std::string &serial = pair.first;
        rs2::pipeline *pipe = pair.second;

        rs2::frameset frames = pipe->wait_for_frames();

        // Get the RGB frame
        rs2::video_frame color_frame = frames.get_color_frame();

        // Get the depth frame
        rs2::depth_frame depth_frame = frames.get_depth_frame();

        if (!color_frame || !depth_frame)
        {
            RCLCPP_WARN(this->get_logger(), "Incomplete frames received from camera %s", serial.c_str());
            continue;
        }

        // Create OpenCV matrices from the frames
        cv::Mat color(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat depth(cv::Size(depth_frame.get_width(), depth_frame.get_height()), CV_16U, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);

        // Save images to disk
        cv::imwrite("rgb_" + serial + ".png", color);
        cv::imwrite("depth_" + serial + ".png", depth);

        // Publish images
        auto rgb_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", color).toImageMsg();
        auto depth_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", depth).toImageMsg();
        rgb_publishers_[serial]->publish(*rgb_msg);
        depth_publishers_[serial]->publish(*depth_msg);

        RCLCPP_INFO(this->get_logger(), "Published images from camera %s", serial.c_str());
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RealSenseMultiCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
