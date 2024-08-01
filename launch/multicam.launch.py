import launch
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        # Declare the launch arguments
        DeclareLaunchArgument(
            'mcpc_camera',
            default_value='Intel Realsense D405',
            description='Camera type for data collection'
        ),
        DeclareLaunchArgument(
            'camera_config_file',
            default_value='rs_405_camera_config.yaml',
            description='Configuration file for the camera'
        ),

        Node(
            package='farmbot_controllers',
            executable='param_conf_server',
            name='param_conf_server',
            output='screen'
        ),
        Node(
            package='farmbot_controllers',
            executable='panel_controller',
            name='panel_controller',
            output='screen'
        ),
        Node(
            package='farmbot_controllers',
            executable='farmbot_controller',
            name='controller',
            output='screen'
        ),
        Node(
            package='farmbot_command_handler',
            executable='motor_command_handler',
            name='motor_command_handler',
            output='screen'
        ),
        Node(
            package='farmbot_command_handler',
            executable='state_command_handler',
            name='state_command_handler',
            output='screen'
        ),
        Node(
            package='farmbot_command_handler',
            executable='device_command_handler',
            name='device_command_handler',
            output='screen'
        ),
        Node(
            package='map_handler',
            executable='map_controller',
            name='map_controller',
            output='screen'
        ),
        Node(
            package='multicam_pointcloud',
            executable='realsense_multicam_node',
            name='realsense_multicam_node',
            output='screen'
        ),
        Node(
            package='multicam_pointcloud',
            executable='data_collection_node.py',
            name='data_collector',
            output='screen',
            parameters=[
                {'mcpc_camera': LaunchConfiguration('mcpc_camera')},
                {'camera_config_file': LaunchConfiguration('camera_config_file')}
            ]
        ),

        # Delay for 15 seconds
        TimerAction(
            period=60.0,
            actions=[
                # Start uart_controller after the delay
                Node(
                    package='farmbot_command_handler',
                    executable='uart_controller',
                    name='uart_controller',
                    output='screen'
                )
            ]
        )
    ])

if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
