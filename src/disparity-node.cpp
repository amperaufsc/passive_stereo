#include "disparity.hpp"


#include<string>

DisparityNode::DisparityNode(sensor_msgs::msg::CameraInfo infoL, sensor_msgs::msg::CameraInfo infoR): Node("node")
{
    std::string left_image_topic = "/left/image_raw";
    std::string right_image_topic = "/right/image_raw";

    left_camera_info = infoL;
    right_camera_info = infoR;

    left_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, left_image_topic);
    right_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, right_image_topic);

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>> (approximate_sync_policy(10), *left_sub, *right_sub);
    syncApproximate->registerCallback(std::bind(&DisparityNode::GrabStereo, this, std::placeholders::_1, std::placeholders::_2));

    publisher = this->create_publisher<sensor_msgs::msg::Image>("disparity_image",10);

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

    auto imgmsg = sensor_msgs::msg::Image();

    

    cv::Mat imgL = cv_ptrLeft->image;
    cv::Mat imgR = cv_ptrRight->image;
    cv::Mat disp, disparity;

    RectifyImages(imgL, imgR);

    cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create(16,9);
    stereo->compute(imgL, imgR, disp);
    disp.convertTo(disparity,CV_32F, 1.0);

    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", disparity).toImageMsg(imgmsg);
    publisher->publish(imgmsg);
    
}
void DisparityNode::RectifyImages(cv::Mat imgL, cv::Mat imgR)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"%d", left_camera_info.height);

    cv::Mat intrinsics_left(3, 3, cv::DataType<double>::type);
    cv::Mat dist_coeffs_left(5, 1, cv::DataType<double>::type);

    cv::Mat intrinsics_right(3, 3, cv::DataType<double>::type);
    cv::Mat dist_coeffs_right(5, 1, cv::DataType<double>::type);
    
    cv::Mat rotation(3, 3, cv::DataType<double>::type);
    cv::Mat translation(3, 1, cv::DataType<double>::type);

    cv::Mat R1,R2,P1,P2,Q;

    cv::Mat left_map1, left_map2;
    cv::Mat right_map1, right_map2;

    cv::Mat rectImgL, rectImgR;

    cv::Size siz;

    intrinsics_left.at<double>(0,0) = left_camera_info.k[0];
    intrinsics_left.at<double>(0,2) = left_camera_info.k[2];
    intrinsics_left.at<double>(1,1) = left_camera_info.k[4];
    intrinsics_left.at<double>(1,2) = left_camera_info.k[5];
    intrinsics_left.at<double>(1,2) = 1;
    
    dist_coeffs_left.at<double>(0) = left_camera_info.k[0];
    dist_coeffs_left.at<double>(1) = left_camera_info.k[2];
    dist_coeffs_left.at<double>(2) = left_camera_info.k[3];
    dist_coeffs_left.at<double>(3) = left_camera_info.k[4];
    dist_coeffs_left.at<double>(4) = left_camera_info.k[5];

    intrinsics_right.at<double>(0,0) = left_camera_info.k[0];
    intrinsics_right.at<double>(0,2) = left_camera_info.k[2];
    intrinsics_right.at<double>(1,1) = left_camera_info.k[4];
    intrinsics_right.at<double>(1,2) = left_camera_info.k[5];
    intrinsics_right.at<double>(1,2) = 1;
    
    dist_coeffs_right.at<double>(0) = left_camera_info.k[0];
    dist_coeffs_right.at<double>(1) = left_camera_info.k[2];
    dist_coeffs_right.at<double>(2) = left_camera_info.k[3];
    dist_coeffs_right.at<double>(3) = left_camera_info.k[4];
    dist_coeffs_right.at<double>(4) = left_camera_info.k[5];

    siz.width = left_camera_info.width;
    siz.height = left_camera_info.height;

    rotation.at<double>(0, 0) = left_camera_info.r[0];
    rotation.at<double>(0, 1) = left_camera_info.r[1];
    rotation.at<double>(0, 2) = left_camera_info.r[2];
    rotation.at<double>(1, 0) = left_camera_info.r[3];
    rotation.at<double>(1, 1) = left_camera_info.r[4];
    rotation.at<double>(1, 2) = left_camera_info.r[5];
    rotation.at<double>(2, 0) = left_camera_info.r[6];
    rotation.at<double>(2, 1) = left_camera_info.r[7];
    rotation.at<double>(2, 2) = left_camera_info.r[8];

    translation.at<double>(0) = left_camera_info.p[3];
    translation.at<double>(1) = left_camera_info.p[7];
    translation.at<double>(2) = left_camera_info.p[11];

    cv::stereoRectify(intrinsics_left, dist_coeffs_left, intrinsics_right, dist_coeffs_right, siz, rotation, translation, R1, R2, P1, P2, Q);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"%d", left_camera_info.height);

    cv::initInverseRectificationMap(intrinsics_left, dist_coeffs_left, R1, P1,siz, CV_32FC1,left_map1, left_map2);
    cv::initInverseRectificationMap(intrinsics_right, dist_coeffs_right, R2, P2,siz, CV_32FC1,right_map1, right_map2);

    cv::remap(imgL, rectImgL, left_map1, left_map2, cv::INTER_LINEAR);
    cv::remap(imgR, rectImgR, right_map1, right_map2, cv::INTER_LINEAR);
}