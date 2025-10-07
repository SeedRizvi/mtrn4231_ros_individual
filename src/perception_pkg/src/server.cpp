#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/srv/start_perception.hpp"

#include <memory>

void start_perception(const std::shared_ptr<custom_msgs::srv::StartPerception::Request> request,
                      std::shared_ptr<custom_msgs::srv::StartPerception::Response> response)
{
    response->success = (request->start) ? true : false;
    response->message = "Perception service started";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request: start=%d",
                request->start);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%d]", response->success);
}

void perform_cv()
{
    // TODO
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("start_perception_server");

    rclcpp::Service<custom_msgs::srv::StartPerception>::SharedPtr service =
        node->create_service<custom_msgs::srv::StartPerception>("start_perception", &start_perception);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to start perception.");

    rclcpp::spin(node);
    rclcpp::shutdown();
}