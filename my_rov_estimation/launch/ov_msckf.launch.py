"""Launch OpenVINS (ov_msckf) for the ROV: front + down cameras, one IMU.

Both cameras are tracked as independent mono cameras (they do not overlap),
so `use_stereo` must be false and `max_cameras` 2. OpenVINS' own
subscribe.launch.py defaults to use_stereo=true, which is wrong for this rig.

Topics are read from the `rostopic` fields of config/kalibr_*.yaml:
    /rov_2/camera_front/image_color   (cam0)
    /rov_2/camera_down/image_color    (cam1)
    /rov_2/imu

Requires the ov_ws workspace (ov_msckf) to be sourced as well.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('my_rov_estimation'),
        'config', 'rov_estimator_config.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=default_config,
                              description='path to estimator_config.yaml'),
        DeclareLaunchArgument('namespace', default_value='ov_msckf'),
        DeclareLaunchArgument('verbosity', default_value='INFO',
                              description='ALL, DEBUG, INFO, WARNING, ERROR, SILENT'),
        DeclareLaunchArgument('save_total_state', default_value='false'),
        Node(
            package='ov_msckf',
            executable='run_subscribe_msckf',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'verbosity': LaunchConfiguration('verbosity'),
                'use_stereo': False,
                'max_cameras': 2,
                'save_total_state': LaunchConfiguration('save_total_state'),
                'config_path': LaunchConfiguration('config_path'),
            }],
        ),
    ])
