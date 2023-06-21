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
        remappings=[('image', '/camera/color/image_raw')],
        parameters=[{
            'gui': teach,
            'objects_path': get_package_share_directory('tutorial_pkg') + '/img_data/' if LaunchConfigurationEquals('teach', 'false') else ''
        }]
    )

    return LaunchDescription([
        teach_arg,
        find_object_2d
    ])