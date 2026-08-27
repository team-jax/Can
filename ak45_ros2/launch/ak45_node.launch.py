import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory('ak45_ros2'),
        'config',
        'ak45_node.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='ak45_node 파라미터 YAML 경로',
        ),
        Node(
            package='ak45_ros2',
            executable='ak45_node',
            name='ak45_node',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
