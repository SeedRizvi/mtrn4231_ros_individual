#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/apple_img_pos_data.hpp"

class ApplePositionTransfer : public rclcpp::Node
{
public:
    ApplePositionTransfer()
        : Node("apple_position_transfer")
    {
        // Create subscription to raw apple detections from OpenCV node
        subscription_ = this->create_subscription<custom_msgs::msg::AppleImgPosData>(
            "apple_detections_raw",
            10,
            std::bind(&ApplePositionTransfer::callback, this, std::placeholders::_1));

        // Create publisher for processed apple detections
        publisher_ = this->create_publisher<custom_msgs::msg::AppleImgPosData>(
            "apple_detections",
            10);

        RCLCPP_INFO(this->get_logger(), "Apple Position Transfer Node initialized");
    }

private:
    void callback(const custom_msgs::msg::AppleImgPosData::SharedPtr msg)
    {
        msg->header.stamp = rclcpp::Clock().now();
        msg->header.frame_id = "camera_optical_frame";
        publisher_->publish(*msg);

        RCLCPP_DEBUG(this->get_logger(), "Transferred apple detection data");
    }

    rclcpp::Subscription<custom_msgs::msg::AppleImgPosData>::SharedPtr subscription_;
    rclcpp::Publisher<custom_msgs::msg::AppleImgPosData>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ApplePositionTransfer>());
    rclcpp::shutdown();
    return 0;
}