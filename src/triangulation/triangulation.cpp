#include "triangulation.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string left_info_topic = "/left/camera_info";

    auto left_camera_info = sensor_msgs::msg::CameraInfo();

    bool left_camera_info_received = false;

    auto info_node = std::make_shared<rclcpp::Node>("camera_info_subscriber");

    left_camera_info_received = rclcpp::wait_for_message<sensor_msgs::msg::CameraInfo>(left_camera_info, info_node,left_info_topic, std::chrono::seconds(5));

    if(left_camera_info_received){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Camera info received");
    }
    else
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No camera info provided");

    return 0;
}