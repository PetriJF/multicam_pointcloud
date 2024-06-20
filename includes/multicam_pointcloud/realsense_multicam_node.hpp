// includes/multicam_pointcloud/realsense_multicam_node.hpp

#ifndef REALSENSE_MULTICAM_NODE_HPP
#define REALSENSE_MULTICAM_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <map>
#include <string>

class RealSenseMultiCameraNode : public rclcpp::Node
{
public:
    RealSenseMultiCameraNode();
    ~RealSenseMultiCameraNode();

private:
    void setup_camera();
    void capture_and_publish();
    
    std::vector<rs2::pipeline> pipelines_;
    std::map<std::string, rs2::pipeline *> serial_to_pipeline_;
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> rgb_publishers_;
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> depth_publishers_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // REALSENSE_MULTICAM_NODE_HPP
