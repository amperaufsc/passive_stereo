#include "triangulation.hpp"

using std::placeholders::_1;

TriangulationNode::TriangulationNode(sensor_msgs::msg::CameraInfo camera_info): Node("triangulation")
{   
    std::string disparity_image_topic = "/disparity/disparity_image";

    disparity_sub_ = this->create_subscription<stereo_msgs::msg::DisparityImage>(disparity_image_topic, 10, std::bind(&TriangulationNode::GrabImage, this, _1));

    baseline_x_fx_ = camera_info.p[3];
    principal_x_ = camera_info.k[2];
    principal_y_ = camera_info.k[5];
    fx_ = camera_info.k[0];
    fy_ = camera_info.k[4];

    RCLCPP_INFO(this->get_logger(), "Baseline: %f", baseline_x_fx_);

    pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);
}
void TriangulationNode::GrabImage(const stereo_msgs::msg::DisparityImage::ConstSharedPtr disparity_image_msg)
{
    auto pointcloudmsg = sensor_msgs::msg::PointCloud2();
    baseline_x_fx_ = disparity_image_msg->t/1000;
    try
    {
         cv_ptr_image = cv_bridge::toCvCopy(disparity_image_msg->image, sensor_msgs::image_encodings::TYPE_32FC1); 
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    int width = cv_ptr_image->image.size().width;
    int height = cv_ptr_image->image.size().height;

    pointcloudmsg.header.stamp = disparity_image_msg->header.stamp;
    pointcloudmsg.header.frame_id = "map";
    pointcloudmsg.height = 1;
    pointcloudmsg.width = width * height;
    pointcloudmsg.is_dense = false;
    pointcloudmsg.fields.resize(3);

    // Populate the fields
    pointcloudmsg.fields[0].name = "x";
    pointcloudmsg.fields[0].offset = 0;
    pointcloudmsg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloudmsg.fields[0].count = 1;

    pointcloudmsg.fields[1].name = "y";
    pointcloudmsg.fields[1].offset = 4;
    pointcloudmsg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloudmsg.fields[1].count = 1;

    pointcloudmsg.fields[2].name = "z";
    pointcloudmsg.fields[2].offset = 8;
    pointcloudmsg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pointcloudmsg.fields[2].count = 1;
    
    pointcloudmsg.point_step = 12; // Size of a single point in bytes (3 floats * 4 bytes/float)
    pointcloudmsg.row_step = pointcloudmsg.point_step * pointcloudmsg.width;
    pointcloudmsg.is_bigendian = true;
    pointcloudmsg.data.resize(pointcloudmsg.point_step * width * height);

    for (int i = 0; i < height; i++)
    {
        for(int j = 0; j < width; j++){
            float disparity = cv_ptr_image->image.at<float>(i,j);
            if(disparity >= 130){
                float z = -baseline_x_fx_ * fx_/ disparity;
                float x = (i - principal_x_) * (z) / fx_;
                float y = (j - principal_y_) * (z) / fy_;

                //RCLCPP_INFO(this->get_logger(), "x: %f, y: %f,z: %f", x,y,z);

                memcpy(&pointcloudmsg.data[(i*width + j)*12], &x, 4);
                memcpy(&pointcloudmsg.data[(i*width + j)*12 + 4], &y, 4);
                memcpy(&pointcloudmsg.data[(i*width + j)*12 + 8], &z, 4);
            }
        }
    }
    //RCLCPP_INFO(this->get_logger(), "Received %f", disparity_image_msg->t);
    pointcloud_publisher_->publish(pointcloudmsg);
}
