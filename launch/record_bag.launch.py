import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


def generate_launch_description():

    # Define config file location
    softsys_joy_config_file = get_share_file(
        package_name='softsys_joy', file_name='config/softsys_joy_config.yaml'
    )

    # tell ros we are using a config file
    softsys_joy_config = DeclareLaunchArgument(
        'softsys_joy_config_file',
        default_value=softsys_joy_config_file,
        description='Path to config file for softsys-joy parameters'
    )

    # define node to launch and parameters to use
    softsys_joy_node = Node(
        package='softsys_joy',
        executable='softsys_joy_node',
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
        parameters=[LaunchConfiguration('softsys_joy_config_file')],
    )

    joy_node = Node(
        package='joy',
        executable = 'joy_node',
        output='screen',
    )

        # Define config file location
    i2cpwm_ros2_config_file = get_share_file(
        package_name='i2cpwm_ros2', file_name='config/i2cpwm_ros2_config.yaml'
    )

    # tell ros we are using a config file
    i2cpwm_ros2_config = DeclareLaunchArgument(
        'i2cpwm_ros2_config_file',
        default_value=i2cpwm_ros2_config_file,
        description='Path to config file for i2cpwm parameters'
    )

    # define node to launch and parameters to use
    i2cpwm_ros2_node = Node(
        package='i2cpwm_ros2',
        executable='i2cpwm_ros2',
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
        parameters=[LaunchConfiguration('i2cpwm_ros2_config_file')],
    )

    # launch gscam
    gscam_launch_dir = get_package_share_directory('gscam');

    gscam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gscam_launch_dir + '/examples/component_pipeline_launch.py')
    )

    return LaunchDescription([
        softsys_joy_config,
        softsys_joy_node,
        i2cpwm_ros2_config,
        i2cpwm_ros2_node,
        # marvelmind_ros2_config,
        # marvelmind_ros2_node,
        joy_node,
        gscam_launch
    ])


