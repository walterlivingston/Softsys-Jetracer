import os
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
            get_package_share_directory('cam_control'),
            'config',
            'base.yaml'
        )

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='cam_control', 
            executable='cam_control', 
            output='screen',
            name="cam_control",
            parameters=[config]
        )
    ])
