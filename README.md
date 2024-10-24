# Disparity Package for ROS2 Humble

## Overview
This package calculates the disparity between a pair of stereo images, which is useful for 3D perception in robotics applications.

## Requirements
- ROS2 Humble
- OpenCV 4.5.4
- Python 3.10

## Installation
1. Clone the repository:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/Projeto-Voris/disparity.git
   ```
2. Install dependencies and build:
   ```bash
   cd ~/ros2_ws
   rosdep install -i --from-path src --rosdistro humble -y
   colcon build --packages-select disparity
   ```

## Usage

### Disparity Node
The disparity node requires a YAML configuration file for [OpenCV StereoBM](https://docs.opencv.org/3.4/d9/dba/classcv_1_1StereoBM.html). The settings should be placed in `disparity/cfg/<file>.yaml`. You can pass the YAML file as a launch argument:

```bash
ros2 launch disparity disparity.launch.py --ros-args yaml_file:=<file_name> left_image:=<left_img_topic> right_image:=<right_img_topic>
```

### PointCloud Node
The PointCloud node constructs a `sensor_msgs::PointCloud2` message from the disparity image.

- **Grayscale Image PointCloud**:
   ```bash
   ros2 launch disparity triangulation.launch.py --ros-args yaml_file:=<file_name> disparity:=<disparity_img_topic>
   ```
- **Colored Image PointCloud**:
   ```bash
   ros2 launch disparity triangulation_rgb.launch.py --ros-args yaml_file:=<file_name> disparity:=<disparity_img_topic> left_image:=<left_img_topic>
   ```

## License
This project is licensed under the Apache-2.0 License.
