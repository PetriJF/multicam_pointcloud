#ifndef REALSENSE_MULTICAM_NODE_HPP_
#define REALSENSE_MULTICAM_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <librealsense2/rs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <unordered_map>

class RealSenseMultiCameraNode : public rclcpp::Node
{
public:
    RealSenseMultiCameraNode();
    ~RealSenseMultiCameraNode();

private:
    void setup_camera();
    void capture_and_publish();
    void toggle_camera_streams(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    std::vector<rs2::pipeline> pipelines_;
    std::unordered_map<std::string, rs2::pipeline*> serial_to_pipeline_;
    std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> rgb_publishers_;
    std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> depth_publishers_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr toggle_service_;
    bool streams_paused_;
};

#endif  // REALSENSE_MULTICAM_NODE_HPP_
