from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Launch arguments
    test_arg = DeclareLaunchArgument(
        'test',
        default_value='0.0',
        description='Test parameter for diff_gps'
    )

    # diff_gps node
    gps_node = Node(
        package='diff_gps',
        executable='diff_gps_node',
        name='diff_gps_node',
        output='screen',
        parameters=[{
            'test': LaunchConfiguration('test')
        }]
    )

    return LaunchDescription([
        test_arg,
        gps_node])
