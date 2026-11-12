from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    mission_lane_node = Node(
        package='lane_detection',
        executable='detection',
        name='lane',
        output='screen'
    )
    
    mission_traffic_light_node = Node(
        package='traffic_light',
        executable='detection',
        name='traffic_light',
        output='screen'
    )
    
    mission_intersection_node = Node(
        package='intersection',
        executable='detection',
        name='intersection',
        output='screen'
    )
    
    mission_construction_node = Node(
        package='construction',
        executable='detection',
        name='construction',
        output='screen'
    )
    
    mission_parking_node = Node(
        package='parking',
        executable='detection',
        name='parking',
        output='screen'
    )
    
    mission_level_crossing_node = Node(
        package='level_crossing',
        executable='detection',
        name='level_crossing',
        output='screen'
    )
    
    mission_tunnel_node = Node(
        package='tunnel',
        executable='detection',
        name='tunnel',
        output='screen'
    )
    
    return LaunchDescription([
        mission_lane_node,
        mission_traffic_light_node,
        mission_intersection_node,
        mission_construction_node,
        mission_parking_node,
        mission_level_crossing_node,
        mission_tunnel_node
    ])
