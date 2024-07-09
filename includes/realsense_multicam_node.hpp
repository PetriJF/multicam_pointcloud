// #ifndef REALSENSE_MULTICAM_NODE_HPP_
// #define REALSENSE_MULTICAM_NODE_HPP_

// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <librealsense2/rs.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>
// #include <unordered_map>
// #include <memory>
// #include <vector>
// #include "farmbot_interfaces/srv/string_rep_req.hpp"

// class RealSenseMultiCameraNode : public rclcpp::Node
// {
// public:
//     RealSenseMultiCameraNode();
//     ~RealSenseMultiCameraNode();

// private:
//     // void setup_camera();
//     // void capture_and_publish();
//     // void toggle_camera_streams(const std::shared_ptr<farmbot_interfaces::srv::StringRepReq::Request> request,
//     //                            std::shared_ptr<farmbot_interfaces::srv::StringRepReq::Response> response);

//     // std::vector<std::shared_ptr<rs2::pipeline>> pipelines_;
//     // std::unordered_map<std::string, std::shared_ptr<rs2::pipeline>> serial_to_pipeline_;
//     // std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> rgb_publishers_;
//     // std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> depth_publishers_;
//     // rclcpp::TimerBase::SharedPtr timer_;
//     // rclcpp::Service<farmbot_interfaces::srv::StringRepReq>::SharedPtr toggle_service_;
//     // bool streams_paused_;

// };

// #endif  // REALSENSE_MULTICAM_NODE_HPP_

#ifndef REALSENSE_MULTICAM_NODE_HPP_
#define REALSENSE_MULTICAM_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <unordered_map>
#include <sensor_msgs/msg/image.hpp>
#include <farmbot_interfaces/srv/string_rep_req.hpp>

class RealSenseMultiCameraNode : public rclcpp::Node
{
public:
    RealSenseMultiCameraNode();
    ~RealSenseMultiCameraNode();

private:
    void setup_camera();
    void read_frames();
    void handle_service(const std::shared_ptr<farmbot_interfaces::srv::StringRepReq::Request> request,
                        std::shared_ptr<farmbot_interfaces::srv::StringRepReq::Response> response);

    std::unordered_map<std::string, std::shared_ptr<rs2::pipeline>> serial_to_pipeline_;
    std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> rgb_publishers_;
    std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> depth_publishers_;
    std::unordered_map<std::string, cv::Mat> color_frames_;
    std::unordered_map<std::string, cv::Mat> depth_frames_;
    bool frames_captured_;

    rclcpp::Service<farmbot_interfaces::srv::StringRepReq>::SharedPtr toggle_service_;
    rclcpp::TimerBase::SharedPtr read_frames_timer_;
};

#endif  // REALSENSE_MULTICAM_NODE_HPP_

