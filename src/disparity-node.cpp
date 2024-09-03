#include "disparity.hpp"


#include<string>

DisparityNode::DisparityNode(): Node("node")
{
    std::string left_image_topic = "/left/image_raw";
    std::string right_image_topic = "/right/image_raw";



    left_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, left_image_topic);
    right_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, right_image_topic);

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>> (approximate_sync_policy(10), *left_sub, *right_sub);
    syncApproximate->registerCallback(std::bind(&DisparityNode::GrabStereo, this, std::placeholders::_1, std::placeholders::_2));

    publisher = this->create_publisher<stereo_msgs::msg::DisparityImage>("disparity_image",10);

}
void DisparityNode::GrabStereo(const ImageMsg::ConstSharedPtr msgLeft, const ImageMsg::ConstSharedPtr msgRight)
{
    try
    {
         cv_ptrLeft = cv_bridge::toCvShare(msgLeft); 
         cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    auto sendmsg = stereo_msgs::msg::DisparityImage();

    RCLCPP_INFO(this->get_logger(), "Image received");

    cv::Mat imgL = cv_ptrLeft->image;
    cv::Mat imgR = cv_ptrRight->image;
    cv::Mat disp;

    cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create(16,9);
    stereo->compute(imgL, imgR, disp);

}