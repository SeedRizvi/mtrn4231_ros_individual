#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/srv/start_perception.hpp"

#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

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
    std::string image_path = "src/perception_pkg/src/";
    std::string image_name = "orchard.png";
    std::string full_image_path = image_path + image_name;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Loading image from: %s", full_image_path.c_str());

    cv::Mat image = cv::imread(full_image_path);
    if (image.empty())
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not open or find the image at: %s", full_image_path.c_str());
        return;
    }

    cv::Mat hsv_image;
    cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

    cv::Mat lower_red_mask, upper_red_mask;

    // Lower red range (0-10)
    cv::Scalar lower_red1 = cv::Scalar(0, 120, 70);
    cv::Scalar upper_red1 = cv::Scalar(10, 255, 255);
    cv::inRange(hsv_image, lower_red1, upper_red1, lower_red_mask);

    // Upper red range (170-180)
    cv::Scalar lower_red2 = cv::Scalar(170, 120, 70);
    cv::Scalar upper_red2 = cv::Scalar(180, 255, 255);
    cv::inRange(hsv_image, lower_red2, upper_red2, upper_red_mask);

    // Combine masks
    cv::Mat red_mask;
    cv::bitwise_or(lower_red_mask, upper_red_mask, red_mask);

    // Apply morph operations to de-noise
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(red_mask, red_mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(red_mask, red_mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Found %ld potential apples", contours.size());

    int apple_count = 0;
    for (size_t i = 0; i < contours.size(); i++)
    {
        double area = cv::contourArea(contours[i]);

        // Filter out very small or very large apples
        if (area > 25 && area < 10000)
        {
            // Calculate moments to find center of mass
            cv::Moments moments = cv::moments(contours[i]);
            int center_x = moments.m10 / moments.m00;
            int center_y = moments.m01 / moments.m00;

            cv::circle(image, cv::Point(center_x, center_y), 5, cv::Scalar(255, 0, 225), -1);
            cv::circle(image, cv::Point(center_x, center_y), 20, cv::Scalar(255, 0, 225), 2);

            ++apple_count;
            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Apple %d: Center at (%d, %d), Area: %.2f",
            //             apple_count, center_x, center_y, area);
        }
    }

    // Save the result image
    std::string output_name = "marked_orchard.png";
    std::string full_output_path = image_path + output_name;
    cv::imwrite(full_output_path, image);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Detected %d apples total. Result saved to %s", apple_count, full_output_path.c_str());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("start_perception_server");

    rclcpp::Service<custom_msgs::srv::StartPerception>::SharedPtr service =
        node->create_service<custom_msgs::srv::StartPerception>("start_perception", &start_perception);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to start perception.");

    perform_cv();

    rclcpp::spin(node);
    rclcpp::shutdown();
}