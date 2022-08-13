import os
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)

def generate_launch_description():
    config = os.path.join(
            get_package_share_directory('cam_control'),
            'config',
            'base.yaml'
        )

    softsys_joy_dir = get_package_share_directory('softsys_joy')
    softsys_base = launch.actions.IncludeLaunchDescription(launch.launch_description_sources.PythonLaunchDescriptionSource(
        softsys_joy_dir + '/launch/base.launch.py'))

    gscam_dir = get_package_share_directory('gscam')
    gscam = launch.actions.IncludeLaunchDescription(launch.launch_description_sources.PythonLaunchDescriptionSource(
        gscam_dir + '/examples/component_pipeline_launch.py'))

    # Define config file location
    marvelmind_ros2_config_file = get_share_file(
        package_name='marvelmind_ros2', file_name='config/marvelmind_ros2_config.yaml'
    )

    # tell ros we are using a config file
    marvelmind_ros2_config = launch.actions.DeclareLaunchArgument(
        'marvelmind_ros2_config_file',
        default_value=marvelmind_ros2_config_file,
        description='Path to config file for marvelmind_ros2_config parameters'
    )

    # define node to launch and parameters to use
    marvelmind_ros2_node = launch_ros.actions.Node(
        package='marvelmind_ros2',
        executable='marvelmind_ros2',
        output='screen',
        arguments=['--ros-args', '--log-level', 'rclcpp:=DEBUG', '--log-level', 'hedgehog_logger:=INFO'],
        parameters=[launch.substitutions.LaunchConfiguration('marvelmind_ros2_config_file')],
    )
    pose_repub_node = launch_ros.actions.Node(
        package='rviz_support', 
        executable='rviz_repub'
    )

    static_tf_pub = launch_ros.actions.Node(
        package='tf2_ros',
        executable="static_transform_publisher",
        name="map_pub",
        arguments="1 0 0 0 0 1 1 inertial map"
    )

    return launch.LaunchDescription([
        softsys_base,
        gscam,
        marvelmind_ros2_config,
        marvelmind_ros2_node,
        pose_repub_node,
        static_tf_pub,
        launch_ros.actions.Node(
            package='cam_control', 
            executable='cam_control', 
            output='screen',
            name="cam_control",
            arguments=['--ros-args', '--log-level', 'rclcpp:=INFO'],
            parameters=[config]
        )
    ])