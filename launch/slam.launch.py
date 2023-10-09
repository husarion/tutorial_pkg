import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            get_package_share_directory("tutorial_pkg"), 'config', 'slam.yaml'
        ),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation/Gazebo clock'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true', description='Launch RViz when true'
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
    )

    rviz_config_path = os.path.join(
        get_package_share_directory("tutorial_pkg"), "rviz", "slam.rviz"
    )
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(use_rviz),
        arguments=["-d", rviz_config_path],
    )

    ld = LaunchDescription()

    ld.add_action(use_sim_time_arg)
    ld.add_action(slam_params_file_arg)
    ld.add_action(use_rviz_arg)

    ld.add_action(slam_node)
    ld.add_action(rviz2_node)

    return ld
