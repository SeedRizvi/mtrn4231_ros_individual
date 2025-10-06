#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/srv/start_perception.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

using namespace std::chrono_literals;

bool parse_args(int argc, char **argv, bool &start_value)
{
    if (argc != 2)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: 'start_perception 1/0' or 'start_perception true/false' for true/false");
        return false;
    }

    std::string arg(argv[1]);
    if (arg == "1" || arg == "true")
    {
        start_value = true;
    }
    else if (arg == "0" || arg == "false")
    {
        start_value = false;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid argument: '%s'. Use '1', '0', 'true', or 'false'", argv[1]);
        return false;
    }

    return true;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    bool start_value;
    if (!parse_args(argc, argv, start_value))
    {
        return 1;
    }

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("start_perception_client");
    rclcpp::Client<custom_msgs::srv::StartPerception>::SharedPtr client =
        node->create_client<custom_msgs::srv::StartPerception>("start_perception");

    auto request = std::make_shared<custom_msgs::srv::StartPerception::Request>();
    request->start = start_value;

    while (!client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Success: %d", result.get()->success);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service start_perception");
    }

    rclcpp::shutdown();
    return 0;
}