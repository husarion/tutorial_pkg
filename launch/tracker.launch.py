from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation/Gazebo clock'
    )

    visualization_arg = DeclareLaunchArgument(
        'visualization', default_value='true', description='Flag to enable visualization mode'
    )

    # Define the 'tracker' node
    tracker = Node(
        package='tutorial_pkg',
        executable='tracker',
        name='tracker',
        remappings=[('/image', '/camera/color/image_raw')],  # Change topic as needed
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                     'visualization': LaunchConfiguration('visualization')}],
    )

    return LaunchDescription([use_sim_time_arg, visualization_arg, tracker])
