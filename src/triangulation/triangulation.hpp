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

// initialize values for StereoSGBM parameters




class DisparityNode : public rclcpp::Node
{
    public:
        DisparityNode(sensor_msgs::msg::CameraInfo infoL, sensor_msgs::msg::CameraInfo infoR);

    private:
        using ImageMsg = sensor_msgs::msg::Image;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_sync_policy;

        void GrabStereo(const sensor_msgs::msg::Image::ConstSharedPtr msgRGB, const sensor_msgs::msg::Image::ConstSharedPtr msgD);
        void RectifyImages(cv::Mat imgL, cv::Mat imgR);
        void CalculateRectificationRemaps();
        void UpdateParameters(const std_msgs::msg::Int16MultiArray::ConstSharedPtr params_message);


        cv::Mat left_map1, left_map2;
        cv::Mat right_map1, right_map2;

        cv::Mat rectImgL, rectImgR;

        cv_bridge::CvImageConstPtr cv_ptrLeft;
        cv_bridge::CvImageConstPtr cv_ptrRight;

        sensor_msgs::msg::CameraInfo left_camera_info;
        sensor_msgs::msg::CameraInfo right_camera_info;

        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> right_sub;

        std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int16MultiArray>> params_sub;

        std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy>> syncApproximate;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr disparity_publisher;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rect_left_publisher;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rect_right_publisher;
};