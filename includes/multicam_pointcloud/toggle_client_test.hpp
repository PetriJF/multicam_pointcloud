#ifndef TOGGLE_CLIENT_TEST_HPP_
#define TOGGLE_CLIENT_TEST_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

class ToggleClientTest : public rclcpp::Node
{
public:
    ToggleClientTest();
    ~ToggleClientTest();

private:
    void send_request();

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
};

#endif  // TOGGLE_CLIENT_TEST_HPP_
