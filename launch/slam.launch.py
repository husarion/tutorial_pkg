import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    map_file = LaunchConfiguration('map_file')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')
    
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(get_package_share_directory("tutorial_pkg"),
                                   'maps', 'map_serialized'),
        description='Full path to the serialized map without extension')

    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("tutorial_pkg"),
                                   'config', 'slam.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam',
        output='screen',
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time,
           'map_file_name': map_file,
           'map_start_pose': [0.0, 0.0, 0.0]}
        ])

    ld = LaunchDescription()

    ld.add_action(use_sim_time_arg)
    ld.add_action(slam_params_file_arg)
    ld.add_action(map_file_arg)
    ld.add_action(slam_node)

    return ld