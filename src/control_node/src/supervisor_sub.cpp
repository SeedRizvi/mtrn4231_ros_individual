#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/command.hpp"
#include "custom_msgs/srv/start_perception.hpp"
#include "commands.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
        : Node("Control_Recv")
    {
        subscription_ = this->create_subscription<custom_msgs::msg::Command>(
            "supervisor_cmds", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

private:
    void start_perception()
    {
        auto client = this->create_client<custom_msgs::srv::StartPerception>("start_perception");
        auto request = std::make_shared<custom_msgs::srv::StartPerception::Request>();
        request->start = true;
        auto request = std::make_shared<custom_msgs::srv::StartPerception::Request>();
        request->start = start_value;
    }
    void topic_callback(const custom_msgs::msg::Command &msg) const
    {

        std::string args_str;
        for (size_t i = 0; i < msg.nargs; ++i)
        {
            if (i > 0)
                args_str += ", ";
            args_str += msg.args[i];
        }
        RCLCPP_INFO(this->get_logger(), "Supervisor Cmd ID: '%ld', %u args: [%s]", msg.command_id, msg.nargs, args_str.c_str());
        switch (msg.command_id)
        {
        case CMD_INVALID:
            RCLCPP_INFO(this->get_logger(), "Invalid command received");
            break;
        case CMD_E_STOP:
            RCLCPP_INFO(this->get_logger(), "Emergency Stop command received");
            break;
        case CMD_START:
            RCLCPP_INFO(this->get_logger(), "Start command received");
            break;
        case CMD_STOP:
            RCLCPP_INFO(this->get_logger(), "Stop command received");
            break;
        case CMD_SHUTDOWN:
            RCLCPP_INFO(this->get_logger(), "Shutdown command received");
            break;
        default:
            RCLCPP_INFO(this->get_logger(), "Unknown command ID: '%ld'", msg.command_id);
            break;
        }
    }
    rclcpp::Subscription<custom_msgs::msg::Command>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}