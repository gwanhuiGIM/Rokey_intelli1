"""fleet_fsm(관제) + safety_alert_bridge(PC3 브릿지) 실행.

    ros2 launch fp_amr_fsm fleet.launch.py
    ros2 launch fp_amr_fsm fleet.launch.py params_file:=/path/to/my.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('fp_amr_fsm'), 'config', 'fleet_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='fleet_fsm 파라미터 yaml 경로'),
        Node(
            package='fp_amr_fsm',
            executable='fleet_fsm',
            name='fleet_fsm_node',
            parameters=[LaunchConfiguration('params_file')],
            output='screen',
        ),
        Node(
            package='fp_amr_fsm',
            executable='safety_alert_bridge',
            name='safety_alert_bridge',
            output='screen',
        ),
    ])
