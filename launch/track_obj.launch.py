from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare the 'visualization' launch argument
    visualization_arg = DeclareLaunchArgument(
        'visualization', default_value='false', description='Flag to enable visualization mode'
    )

    # Define the 'track_obj' node
    track_obj = Node(
        package='tutorial_pkg',
        executable='track_obj',
        name='track_obj',
        remappings=[('/image', '/your/camera/image')],  # Change topic as needed
        condition=UnlessCondition(LaunchConfiguration('visualization')),
    )

    return LaunchDescription([visualization_arg, track_obj])
