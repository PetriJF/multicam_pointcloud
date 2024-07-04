// #include "multicam_pointcloud/realsense_multicam_node.hpp"
// #include <cv_bridge/cv_bridge.h>
// #include <sensor_msgs/msg/image.hpp>
// #include <rclcpp/rclcpp.hpp>


#include "multicam_pointcloud/realsense_multicam_node.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/rclcpp.hpp>

RealSenseMultiCameraNode::RealSenseMultiCameraNode() : Node("realsense_multi_camera"), frames_captured_(false)
{
    setup_camera();
    RCLCPP_INFO(this->get_logger(), "RealSense Multi Camera Node initialized...");

    read_frames_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),  // 5 FPS
        std::bind(&RealSenseMultiCameraNode::read_frames, this));

    RCLCPP_INFO(this->get_logger(), "Wall timer for reading frames created...");

    toggle_service_ = this->create_service<farmbot_interfaces::srv::StringRepReq>(
        "multicam_toggle",
        std::bind(&RealSenseMultiCameraNode::handle_service, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Service created...");
}

void RealSenseMultiCameraNode::setup_camera()
{
    try
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
            auto pipe = std::make_shared<rs2::pipeline>(ctx);
            rs2::config cfg;
            cfg.enable_device(serial);
            cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 5);
            cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 5);
            pipe->start(cfg);
            serial_to_pipeline_[serial] = pipe;

            auto rgb_publisher = this->create_publisher<sensor_msgs::msg::Image>("rgb_" + serial, 10);
            auto depth_publisher = this->create_publisher<sensor_msgs::msg::Image>("depth_" + serial, 10);
            rgb_publishers_[serial] = rgb_publisher;
            depth_publishers_[serial] = depth_publisher;

            RCLCPP_INFO(this->get_logger(), "Camera %s setup complete", serial.c_str());
        }
    }
    catch (const rs2::error &e)
    {
        RCLCPP_ERROR(this->get_logger(), "RealSense error: %s", e.what());
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
    }
}

void RealSenseMultiCameraNode::read_frames()
{
    for (const auto &pair : serial_to_pipeline_)
    {
        const std::string &serial = pair.first;
        auto pipe = pair.second;

        rs2::frameset frames = pipe->wait_for_frames();

        rs2::video_frame color_frame = frames.get_color_frame();
        rs2::depth_frame depth_frame = frames.get_depth_frame();

        if (!color_frame || !depth_frame)
        {
            RCLCPP_WARN(this->get_logger(), "Incomplete frames received from camera %s", serial.c_str());
            continue;
        }

        cv::Mat color(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat depth(cv::Size(depth_frame.get_width(), depth_frame.get_height()), CV_16U, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);

        color_frames_[serial] = color.clone();
        depth_frames_[serial] = depth.clone();
    }

    frames_captured_ = true;
}

void RealSenseMultiCameraNode::handle_service(const std::shared_ptr<farmbot_interfaces::srv::StringRepReq::Request> request,
                                              std::shared_ptr<farmbot_interfaces::srv::StringRepReq::Response> response)
{
    if (request->data == "TAKE")
    {
        if (!frames_captured_)
        {
            RCLCPP_WARN(this->get_logger(), "Frames have not been captured yet.");
            response->data = "FAILED";
            return;
        }

        for (const auto &pair : color_frames_)
        {
            const std::string &serial = pair.first;
            const cv::Mat &color = pair.second;
            const cv::Mat &depth = depth_frames_[serial];

            auto rgb_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", color).toImageMsg();
            auto depth_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", depth).toImageMsg();
            rgb_publishers_[serial]->publish(*rgb_msg);
            depth_publishers_[serial]->publish(*depth_msg);

            RCLCPP_INFO(this->get_logger(), "Published images from camera %s", serial.c_str());
        }

        response->data = "SUCCESS";
    }
    else
    {
        response->data = "FAILED";
    }

    RCLCPP_INFO(this->get_logger(), "Service response: %s", response->data.c_str());
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    {
        auto node = std::make_shared<RealSenseMultiCameraNode>();
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}


// RealSenseMultiCameraNode::RealSenseMultiCameraNode() : Node("realsense_multi_camera"), streams_paused_(false)
// {
//     setup_camera();
//     RCLCPP_INFO(this->get_logger(), "RealSense Multi Camera Node initialized...");

//     timer_ = this->create_wall_timer(
//         std::chrono::milliseconds(200),  // 5 FPS
//         std::bind(&RealSenseMultiCameraNode::capture_and_publish, this));
    
//     RCLCPP_INFO(this->get_logger(), "Wall timer created...");

//     toggle_service_ = this->create_service<farmbot_interfaces::srv::StringRepReq>(
//         "toggle_camera_streams",
//         std::bind(&RealSenseMultiCameraNode::toggle_camera_streams, this, std::placeholders::_1, std::placeholders::_2));

//     RCLCPP_INFO(this->get_logger(), "Service created...");
// }

// RealSenseMultiCameraNode::~RealSenseMultiCameraNode()
// {
//     // Stop the timer
//     if (timer_) {
//         timer_->cancel();
//     }

//     // Stop all pipelines
//     for (auto &pipe : pipelines_)
//     {
//         pipe->stop();
//     }

//     RCLCPP_INFO(this->get_logger(), "RealSense Multi Camera Node shutting down...");
// }

// void RealSenseMultiCameraNode::setup_camera()
// {
//     try
//     {
//         rs2::context ctx;
//         std::vector<std::string> serials;
//         for (auto &&dev : ctx.query_devices())
//         {
//             std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
//             serials.push_back(serial);
//             RCLCPP_INFO(this->get_logger(), "Connected camera serial number: %s", serial.c_str());
//         }

//         for (const auto &serial : serials)
//         {
//             auto pipe = std::make_shared<rs2::pipeline>(ctx);
//             rs2::config cfg;
//             cfg.enable_device(serial);
//             cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 5);
//             cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 5);
//             pipe->start(cfg);
//             pipelines_.push_back(pipe);
//             serial_to_pipeline_[serial] = pipe;

//             auto rgb_publisher = this->create_publisher<sensor_msgs::msg::Image>("rgb_" + serial, 10);
//             auto depth_publisher = this->create_publisher<sensor_msgs::msg::Image>("depth_" + serial, 10);
//             rgb_publishers_[serial] = rgb_publisher;
//             depth_publishers_[serial] = depth_publisher;

//             RCLCPP_INFO(this->get_logger(), "Camera %s setup complete", serial.c_str());
//         }
//     }
//     catch (const rs2::error &e)
//     {
//         RCLCPP_ERROR(this->get_logger(), "RealSense error: %s", e.what());
//     }
//     catch (const std::exception &e)
//     {
//         RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
//     }
// }

// void RealSenseMultiCameraNode::capture_and_publish()
// {
//     if (streams_paused_) {
//         RCLCPP_INFO(this->get_logger(), "Streams are paused...");
//         return;
//     }

//     RCLCPP_INFO(this->get_logger(), "Capturing and publishing frames...");

//     for (const auto &pair : serial_to_pipeline_)
//     {
//         const std::string &serial = pair.first;
//         auto pipe = pair.second;

//         rs2::frameset frames = pipe->wait_for_frames();

//         // Get the RGB frame
//         rs2::video_frame color_frame = frames.get_color_frame();

//         // Get the depth frame
//         rs2::depth_frame depth_frame = frames.get_depth_frame();

//         if (!color_frame || !depth_frame)
//         {
//             RCLCPP_WARN(this->get_logger(), "Incomplete frames received from camera %s", serial.c_str());
//             continue;
//         }

//         // Create OpenCV matrices from the frames
//         cv::Mat color(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);
//         cv::Mat depth(cv::Size(depth_frame.get_width(), depth_frame.get_height()), CV_16U, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);

//         // Publish images
//         auto rgb_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", color).toImageMsg();
//         auto depth_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", depth).toImageMsg();
//         rgb_publishers_[serial]->publish(*rgb_msg);
//         depth_publishers_[serial]->publish(*depth_msg);

//         RCLCPP_INFO(this->get_logger(), "Published images from camera %s", serial.c_str());
//     }
// }

// void RealSenseMultiCameraNode::toggle_camera_streams([[maybe_unused]] const std::shared_ptr<farmbot_interfaces::srv::StringRepReq::Request> request,
//                                                      std::shared_ptr<farmbot_interfaces::srv::StringRepReq::Response> response)
// {
//     streams_paused_ = !streams_paused_;
//     response->data = "SUCCESS";
//     RCLCPP_INFO(this->get_logger(), response->data.c_str());
// }

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     {
//         auto node = std::make_shared<RealSenseMultiCameraNode>();
//         rclcpp::spin(node);
//     }
//     rclcpp::shutdown();
//     return 0;
// }
