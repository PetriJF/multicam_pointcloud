#include "toggle_client_test.hpp"

ToggleClientTest::ToggleClientTest() : Node("toggle_client_test")
{
    client_ = this->create_client<farmbot_interfaces::srv::StringRepReq>("toggle_camera_streams");

    // Send the request immediately
    send_request();
}

ToggleClientTest::~ToggleClientTest()
{
    RCLCPP_INFO(this->get_logger(), "ToggleClientTest node shutting down...");
}

void ToggleClientTest::send_request()
{
    RCLCPP_INFO(this->get_logger(), "Sending toggle request");

    auto request = std::make_shared<farmbot_interfaces::srv::StringRepReq::Request>();

    if (!client_->wait_for_service(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Service not available after waiting");
        return;
    }

    auto future = client_->async_send_request(request);

    try
    {
        // Wait for the result (timeout set to 10 seconds)
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, std::chrono::seconds(10)) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Response: %s", response->data.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Service call timed out");
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
    }

    // Schedule node shutdown
    rclcpp::shutdown();
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    {
        auto node = std::make_shared<ToggleClientTest>();
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}
