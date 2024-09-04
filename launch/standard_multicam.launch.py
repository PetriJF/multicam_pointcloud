import launch
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

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
            default_value='2',
            description='0 non-periodic, 1 - daily reading, 2 -morning and evening readings'
        ),

        # Nodes that are always launched
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

        # Launch the Luxonis OAK-D Lite camera node
        Node(
            package='multicam_pointcloud',
            executable='luxonis_multicam_node',
            name='luxonis_multicam',
            output='screen',
            parameters=[
                {'mcpc_camera': LaunchConfiguration('mcpc_camera')},
                {'camera_config_file': LaunchConfiguration('camera_config_file')},
                {'image_save_path': LaunchConfiguration('image_save_path')},
                {'daily_measurement_count': LaunchConfiguration('daily_measurement_count')}
            ]
        ),

        # Delay for 15 seconds
        TimerAction(
            period=15.0,
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
