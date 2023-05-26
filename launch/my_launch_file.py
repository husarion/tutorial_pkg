from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    my_node = Node(
        package="tutorial_pkg",
        executable="my_first_node",
        name="my_node",
        remappings=[("/image", "/your/camera/name")],
        parameters=[{"timer_period_s": 1}]
    )

    return LaunchDescription([my_node])