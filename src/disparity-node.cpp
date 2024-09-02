#include "disparity.hpp"

#include<opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>
#include<string>

DisparityNode::DisparityNode(ros::NodeHandle &node_handle): Node("node")
{
    std::string left_image_topic = "/left/image_raw";
    std::string right_image_topic = "/right/image_raw";


    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Here");

    left_sub = new message_filters::Subscriber<ImageMsg>(std::shared_ptr<rclcpp::Node>(this), left_image_topic);
    right_sub = new message_filters::Subscriber<ImageMsg>(std::shared_ptr<rclcpp::Node>(this), right_image_topic);

    syncApproximate = new message_filters::Synchronizer<approximate_sync_policy> (approximate_sync_policy(10), *left_sub, *right_sub);
    syncApproximate->registerCallback(boost::bind(&DisparityNode::GrabStereo), node_handle, _1, _2);

    publisher = this->create_publisher<stereo_msgs::msg::DisparityImage>("disparity_image",10);

}
void DisparityNode::GrabStereo(const ImageMsg::SharedPtr msgLeft, const ImageMsg::SharedPtr msgRight)
{
    RCLCPP_INFO(this->get_logger(), "Here");
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
    //cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create();
    

}