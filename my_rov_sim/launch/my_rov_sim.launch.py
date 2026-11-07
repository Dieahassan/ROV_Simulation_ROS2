import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # description_path = get_package_share_directory('ricketts_sim')
    # xacro_path = os.path.join(description_path, "urdf", "ricketts.urdf.xacro")
    # robot_description_command = Command(['xacro ', xacro_path])

    # Declare launch arguments
    # robot_name_arg = DeclareLaunchArgument(
    #     'robot_name',
    #     default_value='bluerov',
    #     description='Name of the robot'
    # )
    rov_sim__pkg = get_package_share_directory('my_rov_sim')
    rov_data=os.path.join(rov_sim__pkg ,"data")
    rov_scenario_desc=os.path.join(rov_sim__pkg ,"scenarios",'new_world_1.scn')
    # Group action with namespace
    namespace_action = GroupAction(
        actions=[
            # Include the simulator launch file
            IncludeLaunchDescription(
                PathJoinSubstitution([
                    FindPackageShare('stonefish_ros2'), 'launch', 'stonefish_simulator.launch.py'
                ]),
                launch_arguments={
                    'simulation_data': rov_data, 
                    'scenario_desc': rov_scenario_desc,
                    'simulation_rate': '100.0',
                    'window_res_x': '1200',
                    'window_res_y': '800',
                    'rendering_quality': 'low'
                }.items()
            ),
        ]
    ) 
    
    return LaunchDescription([
        namespace_action,
        ])