#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/command.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
	MinimalPublisher()
		: Node("Supervisor"), count_(0)
	{
		publisher_ = this->create_publisher<custom_msgs::msg::Command>("supervisor_cmds", 10);
		timer_ = this->create_wall_timer(
			1500ms, std::bind(&MinimalPublisher::timer_callback, this));
	}

private:
	void timer_callback()
	{
		auto message = custom_msgs::msg::Command();
		message.nargs = 2;
		message.command_id = 2;
		message.args = {"debug", "full"};

		std::string args_str;
		for (size_t i = 0; i < message.nargs; ++i)
		{
			if (i > 0)
				args_str += ", ";
			args_str += message.args[i];
		}
		RCLCPP_INFO(this->get_logger(), "Command: '%ld' '%u' '[%s]'", message.command_id, message.nargs, args_str.c_str());
		publisher_->publish(message);
	}
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<custom_msgs::msg::Command>::SharedPtr publisher_;
	size_t count_;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MinimalPublisher>());
	rclcpp::shutdown();
	return 0;
}