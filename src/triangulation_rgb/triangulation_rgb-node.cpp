#include "triangulation_rgb.hpp"

using std::placeholders::_1;

TriangulationNode::TriangulationNode(sensor_msgs::msg::CameraInfo camera_info): Node("triangulation_rgb") {
    std::string disparity_image_topic = "/disparity/disparity_image";
    std::string left_image_topic = "/left/image_raw";

    disparity_sub = std::make_shared<message_filters::Subscriber<stereo_msgs::msg::DisparityImage> >(
        this, disparity_image_topic);
    left_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image> >(this, left_image_topic);

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(
        approximate_sync_policy(50), *disparity_sub, *left_sub);

    syncApproximate->registerCallback(std::bind(&TriangulationNode::GrabImages, this, std::placeholders::_1,
                                                std::placeholders::_2));

    baseline_ = camera_info.p[3];
    principal_x_ = camera_info.k[2];
    principal_y_ = camera_info.k[5];
    fx_ = camera_info.k[0];
    fy_ = camera_info.k[4];
    f_ = (camera_info.k[0] + camera_info.k[4]) / 2;

    RCLCPP_INFO(this->get_logger(), "Baseline: %f", baseline_);

    pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);
}

void TriangulationNode::GrabImages(const ImageMsg::ConstSharedPtr disp_msg,
                                   const sensor_msgs::msg::Image::ConstSharedPtr left_msg) {
    // PointCloud2 message
    sensor_msgs::msg::PointCloud2 pointcloudmsg;
    baseline_ = 10*disp_msg->t;

    RCLCPP_INFO(this->get_logger(), "Processing disp at timestamp: %d", disp_msg->header.stamp.sec);
    RCLCPP_INFO(this->get_logger(), "Processing left at timestamp: %d", left_msg->header.stamp.sec);

    // Convert disparity and left images to OpenCV format
    try {
        cv_ptr_disp = cv_bridge::toCvCopy(disp_msg->image, sensor_msgs::image_encodings::TYPE_32FC1);
        cv_ptr_left = cv_bridge::toCvShare(left_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    int width = cv_ptr_disp->image.cols;
    int height = cv_ptr_disp->image.rows;

    // Set PointCloud2 header
    pointcloudmsg.header.stamp = this->get_clock()->now();
    pointcloudmsg.header.frame_id = "depth_map";
    pointcloudmsg.width = 1;  // Points per row
    pointcloudmsg.height = height*width; // Number of rows
    pointcloudmsg.is_dense = false; // Allow NaN points
    pointcloudmsg.is_bigendian = false; // Little-endian (default for most systems)

    // Add fields for x, y, z, and rgb
    sensor_msgs::PointCloud2Modifier modifier(pointcloudmsg);
    modifier.setPointCloud2Fields(4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);

    // Set point step and row step
    pointcloudmsg.point_step = 16;  // 4 fields * 4 bytes (x, y, z, rgb)
    pointcloudmsg.row_step = pointcloudmsg.point_step * pointcloudmsg.width;

    // Resize the point cloud data array
    pointcloudmsg.data.resize(pointcloudmsg.row_step * pointcloudmsg.height);
    float* disparity_data = (float*)cv_ptr_disp->image.data;
    #pragma omp parallel for
    // Iterate through disparity map and generate point cloud
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            // float disparity = cv_ptr_disp->image.at<float>(i, j);
            float disparity = disparity_data[i * width + j];
            if (disparity >= 0) {
                // Compute 3D coordinates from disparity
                float z = baseline_ * fx_ / (disparity);
                float x = (j - principal_x_) * z / fx_;
                float y = (i - principal_y_) * z / fy_;

                // Get RGB from the left image
                cv::Vec3b bgr = cv_ptr_left->image.at<cv::Vec3b>(i, j);
                uint8_t r = bgr[2], g = bgr[1], b = bgr[0];
                // Packed RGB value
                uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

                // Copy x, y, z, and rgb to the point cloud message data
                memcpy(&pointcloudmsg.data[(i * width + j) * 16], &x, sizeof(float));
                memcpy(&pointcloudmsg.data[(i * width + j) * 16 + 4], &y, sizeof(float));
                memcpy(&pointcloudmsg.data[(i * width + j) * 16 + 8], &z, sizeof(float));
                memcpy(&pointcloudmsg.data[(i * width + j) * 16 + 12], &rgb, sizeof(uint32_t));
            }
        }
    }

    // RCLCPP_INFO(this->get_logger(), "Publishing point cloud with width: %d, height: %d", pointcloudmsg.width, pointcloudmsg.height);
    pointcloud_publisher_->publish(pointcloudmsg);
}

