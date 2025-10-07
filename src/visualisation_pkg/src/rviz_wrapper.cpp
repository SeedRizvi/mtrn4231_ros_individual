#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/apple_img_pos_data.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"

class RVizWrapper : public rclcpp::Node
{
public:
    RVizWrapper()
        : Node("rviz_wrapper")
    {
        // Create subscription to apple detections
        subscription_ = this->create_subscription<custom_msgs::msg::AppleImgPosData>(
            "apple_detections",
            10,
            std::bind(&RVizWrapper::apple_detection_callback, this, std::placeholders::_1));

        // Create publisher for visualization markers
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "apple_detection_markers",
            10);

        RCLCPP_INFO(this->get_logger(), "RViz Wrapper Node initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribed to: apple_detections");
        RCLCPP_INFO(this->get_logger(), "Publishing to: apple_detection_markers");
    }

private:
    void apple_detection_callback(const custom_msgs::msg::AppleImgPosData::SharedPtr msg)
    {
        if (msg->count == 0 || msg->detections.empty())
        {
            RCLCPP_DEBUG(this->get_logger(), "No apples detected, clearing markers");
            publish_empty_marker_array();
            return;
        }

        auto marker_array = std::make_shared<visualization_msgs::msg::MarkerArray>();

        // Create markers for each detected apple
        for (size_t i = 0; i < msg->detections.size(); ++i)
        {
            visualization_msgs::msg::Marker marker;

            // Marker header - Override to use map frame
            marker.header.stamp = msg->header.stamp;
            marker.header.frame_id = msg->header.frame_id;
            marker.ns = "apple_detections";
            marker.id = static_cast<int>(i); // Use array index as marker ID
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Marker position - Use raw coordinates from detections
            marker.pose.position.x = msg->detections[i].x / 100;
            marker.pose.position.y = msg->detections[i].y / 100;
            marker.pose.position.z = msg->detections[i].z / 100;
            marker.pose.orientation.w = 1.0; // No rotation

            // Marker scale (size of the sphere)
            marker.scale.x = 0.1; // 20cm diameter - more visible
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;

            // Marker color (red for apples)
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.8; // Slightly transparent

            // Permanent marker (no lifetime)
            marker.lifetime = rclcpp::Duration::from_seconds(0.0);

            marker_array->markers.push_back(marker);
        }

        // Publish the marker array
        marker_publisher_->publish(*marker_array);
        // ! Needs to sleep otherwise Rviz doesn't pick it up
        rclcpp::sleep_for(std::chrono::milliseconds(150));
        RCLCPP_INFO(this->get_logger(), "Published %d apple markers to RViz",
                    static_cast<int>(marker_array->markers.size()));
    }

    void publish_empty_marker_array()
    {
        auto marker_array = std::make_shared<visualization_msgs::msg::MarkerArray>();

        // Create a DELETE action marker to clear all previous markers
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header.stamp = rclcpp::Clock().now();
        clear_marker.header.frame_id = "camera_optical_frame";
        clear_marker.ns = "apple_detections";
        clear_marker.id = 0;
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;

        marker_array->markers.push_back(clear_marker);
        marker_publisher_->publish(*marker_array);
    }

    rclcpp::Subscription<custom_msgs::msg::AppleImgPosData>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RVizWrapper>());
    rclcpp::shutdown();
    return 0;
}