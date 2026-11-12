from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    core_node = Node(
        package='core',
        executable='core',
        name='core',
        output='screen'
    )
    
    return LaunchDescription([
        core_node
    ])
