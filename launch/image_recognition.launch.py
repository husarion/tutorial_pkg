from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    teach = LaunchConfiguration('teach', default='false')

    teach_arg = DeclareLaunchArgument(
        'teach',
        default_value='false',
        description='Flag to enable teaching mode'
    )

    # Object detection
    find_object_2d = Node(
        package='find_object_2d',
        executable='find_object_2d',
        name='find_object_2d',
        remappings=[('image', '/your/camera/image')], # Change topic
        parameters=[{
            'gui': teach,
            'objects_path': get_package_share_directory('tutorial_pkg') + '/img_data/' if LaunchConfigurationEquals('teach', 'false') else ''
        }]
    )

    # Object tracking
    track_obj = Node(
        package='tutorial_pkg',
        executable='track_obj',
        name='track_obj',
        condition=LaunchConfigurationEquals("teach", "false"),
    )

    ld = LaunchDescription()

    ld.add_action(teach_arg)
    ld.add_action(find_object_2d)
    ld.add_action(track_obj)

    return ld