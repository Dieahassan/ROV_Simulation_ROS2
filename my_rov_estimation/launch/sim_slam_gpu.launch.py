"""Stonefish simulation on the NVIDIA GPU + OpenVINS SLAM, in one launch.

What it starts
  1. my_rov_sim/my_rov_sim_teleop.launch.py (the simulator, EKF, TF nodes) with
     PRIME render offload so Stonefish renders on the RTX instead of the Intel iGPU.
  2. After `slam_delay` seconds, my_rov_estimation/ov_msckf.launch.py
     (OpenVINS, front + down mono cameras, the latest config/ files).

RViz opens by default with my_rov_sim/cfg/my_rov_slam.rviz (front/down cameras,
OpenVINS track image, odometry, TF, robot model). The AprilTag node is OFF by
default. Both slow the camera publishing (RViz draws the images on the same GPU,
AprilTag subscribes to the front camera): use rviz:=false to turn RViz off, and
apriltag:=true to turn the AprilTag node on.

Usage
    source /opt/ros/humble/setup.bash
    source /media/deyaa/Dx100/my_ws/ov_ws/install/setup.bash     # OpenVINS (before rov_ws)
    source /media/deyaa/Dx100/SW_Output/rov_ws/install/setup.bash
    ros2 launch my_rov_estimation sim_slam_gpu.launch.py

Arguments
    gpu:=true|false          NVIDIA offload for the simulator (default true)
    slam:=true|false         start OpenVINS (default true)
    slam_delay:=20.0         seconds to wait for the simulator before OpenVINS
    rviz:=true|false         RViz with my_rov_slam.rviz (default true)
    apriltag:=false          AprilTag estimation node from the sim launch
    verbosity:=INFO          OpenVINS verbosity

OpenVINS initialises only after the ROV moves (it waits for an accel jerk),
so drive the ROV a little after it starts.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            LogInfo, SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(package, launch_file, args=None):
    path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=(args or {}).items())


def generate_launch_description():
    gpu = IfCondition(LaunchConfiguration('gpu'))

    # scoped=True: the include's launch_arguments (rviz:=false) must not leak out and
    # override the user's rviz:=... when the RViz node below is evaluated.
    sim = GroupAction(scoped=True, actions=[
        _include('my_rov_sim', 'my_rov_sim_teleop.launch.py', {
            # RViz is started below with the SLAM config, so the sim's own copy stays off.
            'rviz': 'false',
            'apriltag': LaunchConfiguration('apriltag'),
        }),
    ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_slam',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('my_rov_sim'),
                                      'cfg', 'my_rov_slam.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    slam = TimerAction(
        period=LaunchConfiguration('slam_delay'),
        condition=IfCondition(LaunchConfiguration('slam')),
        actions=[
            LogInfo(msg='Starting OpenVINS (ov_msckf). Move the ROV so it can initialise.'),
            _include('my_rov_estimation', 'ov_msckf.launch.py', {
                'verbosity': LaunchConfiguration('verbosity'),
            }),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('gpu', default_value='true',
                              description='render the simulator on the NVIDIA GPU (PRIME offload)'),
        DeclareLaunchArgument('slam', default_value='true',
                              description='start OpenVINS after the simulator'),
        DeclareLaunchArgument('slam_delay', default_value='20.0',
                              description='seconds to wait before starting OpenVINS'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='launch RViz with cfg/my_rov_slam.rviz'),
        DeclareLaunchArgument('apriltag', default_value='false',
                              description='launch the AprilTag estimation node'),
        DeclareLaunchArgument('verbosity', default_value='INFO',
                              description='OpenVINS verbosity: ALL, DEBUG, INFO, WARNING, ERROR, SILENT'),
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1', condition=gpu),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia', condition=gpu),
        sim,
        rviz,
        slam,
    ])
