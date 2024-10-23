from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.substitutions import LaunchConfiguration as LaunchConfig

def generate_launch_description():
    
    return LaunchDescription([
        LaunchArg( 'left_image', default_value=['/stereo_left']),
        LaunchArg( 'right_image', default_value=['/stereo_right']),
        LaunchArg( 'left_info', default_value=['/stereo_left/camera_info']),
        LaunchArg( 'right_info', default_value=['/stereo_right/camera_info']),
        LaunchArg( 'stereo_params', default_value=['/sm2/disparity/stereo_params']),
        LaunchArg('disparity', default_value=['/sm2/disparity/disparity_image']),

       
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
        ),
        Node(
            package='disparity',
            namespace='/sm2/triangulation',
            executable='triangulation',
            name='triangulation',
            remappings=[
                ('/disparity/disparity_image',  LaunchConfig('disparity')),
                ('/left/camera_info', LaunchConfig('left_info'))
            ]
        )

    ])


