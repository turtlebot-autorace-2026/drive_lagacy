"""Existing core and five missions, using the simulator's camera/scan topics."""
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    missions = [Node(package=package, executable='detection', output='screen',
                     parameters=[{'use_sim_time': True}])
                for package in ['lane_detection', 'traffic_light', 'intersection',
                                'construction', 'parking', 'level_crossing']]
    core = Node(package='core', executable='core', output='screen',
                parameters=[{'use_sim_time': True, 'stop_before_tunnel': True}])
    return LaunchDescription(missions + [TimerAction(period=2.0, actions=[core])])
