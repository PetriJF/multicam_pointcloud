import launch
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        # Declare the launch arguments
        DeclareLaunchArgument(
            'mcpc_camera',
            default_value='Luxonis OAK-D Lite',
            description='Camera type for data collection'
        ),
        DeclareLaunchArgument(
            'camera_config_file',
            default_value='luxonis_oak_d_lite_camera_config.yaml',
            description='Configuration file for the camera'
        ),
        DeclareLaunchArgument(
            'image_save_path',
            default_value='',
            description='Save location for the images'
        ),
        DeclareLaunchArgument(
            'daily_measurement_count',
            default_value='0',
            description='0 non-periodic, 1 - daily reading, 2 -morning and evening readings'
        ),
        Node(
            package='multicam_pointcloud',
            executable='data_collection_node.py',
            name='data_collector',
            output='screen',
            parameters=[
                {'mcpc_camera': LaunchConfiguration('mcpc_camera')},
                {'camera_config_file': LaunchConfiguration('camera_config_file')},
                {'image_save_path': LaunchConfiguration('image_save_path')},
                {'daily_measurement_count': LaunchConfiguration('daily_measurement_count')}
            ]
        )
    ])

if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
