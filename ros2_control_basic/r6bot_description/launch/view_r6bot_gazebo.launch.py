from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Specify the package and URDF file name
    urdf_file_name = 'r6bot.urdf.xacro'  # Your URDF/Xacro file
    package_name = 'r6bot_description'   # Your package name
    
    urdf_path = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        urdf_file_name
    )

    # Generate the robot description from the Xacro file
    xacro_command = ['xacro', urdf_path]
    robot_description_content = os.popen(' '.join(xacro_command)).read()

    # Use the robot_state_publisher to publish joint states and transforms
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # Start Gazebo and load an empty world
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Add a command to spawn the robot in Gazebo
    spawn_robot = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
        '-topic', '/robot_description', 
        '-entity', 'r6bot',
        '-x', '0', '-y', '0', '-z', '0.1'  # Specify the position to spawn the robot
    ],
    output='screen'
)

    return LaunchDescription([
        # Launch Gazebo
        gazebo,
        # Launch robot_state_publisher
        robot_state_publisher_node,
        # Spawn the robot in Gazebo
        spawn_robot
    ])
