#ifndef TOGGLE_CLIENT_TEST_HPP_
#define TOGGLE_CLIENT_TEST_HPP_

#include <rclcpp/rclcpp.hpp>
#include "farmbot_interfaces/srv/string_rep_req.hpp"

class ToggleClientTest : public rclcpp::Node
{
public:
    ToggleClientTest();
    ~ToggleClientTest();

private:
    void send_request();

    rclcpp::Client<farmbot_interfaces::srv::StringRepReq>::SharedPtr client_;
};

#endif  // TOGGLE_CLIENT_TEST_HPP_
