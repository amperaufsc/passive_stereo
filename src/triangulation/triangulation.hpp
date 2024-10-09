#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>

#include <std_msgs/msg/int16_multi_array.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <stereo_msgs/msg/disparity_image.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>

class TriangulationNode : public rclcpp::Node
{
    public:
        TriangulationNode(sensor_msgs::msg::CameraInfo camera_info);

    private:
        using ImageMsg = sensor_msgs::msg::Image;

        void GrabImage(const ImageMsg::ConstSharedPtr disparity_image_msg);

        float baseline_x_fx_, principal_x_, principal_y_, fx_, fy_;

        cv_bridge::CvImageConstPtr cv_ptr_image;

        sensor_msgs::msg::CameraInfo camera_info_;
        std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::Image>> disparity_sub_;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
};