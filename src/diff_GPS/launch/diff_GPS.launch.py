import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare any launch arguments
        DeclareLaunchArgument(
            'test',
            default_value='0.0'
        ),
        
        # Main Node
        Node(
            package='diff_GPS',
            executable='diff_GPS_node',
            name='diff_GPS_node',
            output='screen',
            parameters=[{'test': launch.substitutions.LaunchConfiguration('test')}]
        )
    ])
