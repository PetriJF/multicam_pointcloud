import launch
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    mcpc_camera = LaunchConfiguration('mcpc_camera')

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
        
        # Conditional node launch based on mcpc_camera value
        Node(
            condition=LaunchConfigurationEquals('mcpc_camera', 'Intel Realsense D405'),
            package='multicam_pointcloud',
            executable='realsense_multicam_node',
            name='realsense_multicam_node',
            output='screen'
        ),
        Node(
            condition=LaunchConfigurationEquals('mcpc_camera', 'Luxonis OAK-D Lite'),
            package='multicam_pointcloud',
            executable='luxonis_multicam_node.py',
            name='luxonis_multicam',
            output='screen'
        ),

        Node(
            package='multicam_pointcloud',
            executable='data_collection_node.py',
            name='data_collector',
            output='screen',
            parameters=[
                {'mcpc_camera': mcpc_camera},
                {'camera_config_file': LaunchConfiguration('camera_config_file')},
                {'image_save_path': '~/farmbot_images/'},
                {'daily_measurement_count': 0}
            ]
        )
    ])

if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
