#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <stereo_msgs/msg/disparity_image.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/core/utility.hpp"

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

        cv::Mat left_map1, left_map2;
        cv::Mat right_map1, right_map2;

        cv_bridge::CvImageConstPtr cv_ptrLeft;
        cv_bridge::CvImageConstPtr cv_ptrRight;

        sensor_msgs::msg::CameraInfo left_camera_info;
        sensor_msgs::msg::CameraInfo right_camera_info;

        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> right_sub;

        std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy>> syncApproximate;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
};