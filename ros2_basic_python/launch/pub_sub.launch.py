from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Publisher Node
        Node(
            package='ros2_basic_python',
            executable='talker_node',
            output='screen',
        ),
        
        # Subscriber Node 
         Node(
            package='ros2_basic_python',
            executable='listener_node',
            output='screen',
        ),

    ])



