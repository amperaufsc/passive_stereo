
#include <ros/ros.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "stereo_msgs/msg/disparity_image.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <cv_bridge/cv_bridge.h>

class DisparityNode : public rclcpp::Node
{
    public:
        DisparityNode();

    private:
        using ImageMsg = sensor_msgs::msg::Image;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_sync_policy;

        void GrabStereo(const sensor_msgs::msg::Image::SharedPtr msgRGB, const sensor_msgs::msg::Image::SharedPtr msgD);

        cv_bridge::CvImageConstPtr cv_ptrLeft;
        cv_bridge::CvImageConstPtr cv_ptrRight;

        message_filters::Subscriber<sensor_msgs::msg::Image>  *left_sub;
        message_filters::Subscriber<sensor_msgs::msg::Image>  *right_sub;

        message_filters::Synchronizer<approximate_sync_policy> *syncApproximate;

        rclcpp::Publisher<stereo_msgs::msg::DisparityImage>::SharedPtr publisher;
};