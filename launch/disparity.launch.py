from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.substitutions import LaunchConfiguration as LaunchConfig

def generate_launch_description():
    
    return LaunchDescription([
        LaunchArg( 'left_image', default_value=['/sm2_left/image_raw']),
        LaunchArg( 'right_image', default_value=['/sm2_right/image_raw']),
        LaunchArg( 'left_info', default_value=['/sm2_left/camera_info']),
        LaunchArg( 'right_info', default_value=['/sm2_right/camera_info']),
        LaunchArg( 'stereo_params', default_value=['/sm2/disparity/stereo_params']),

       
        Node(
            package='disparity',
            namespace='/sm2/disparity',
            executable='disparity',
            name='disparity',
            remappings=[
                ('/left/image_raw',  LaunchConfig('left_image')),
                ('/right/image_raw', LaunchConfig('right_image')),
                ('/left/camera_info', LaunchConfig('left_info')),
                ('/right/camera_info', LaunchConfig('right_info')),
                ('/params', LaunchConfig('stereo_params'))
            ]
        )

    ])


