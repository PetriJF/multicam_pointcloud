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
