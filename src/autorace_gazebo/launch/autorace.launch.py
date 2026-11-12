"""Gazebo Classic world for the AutoRace v014 map with a TurtleBot3.

Default world: floor + mission devices + static props + detection regions, with the
race manager running. Default spawn is just behind the red start line, in the start
straight, facing the start line (yellow lane on the robot's left).
"""
import os
import math
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# The regions plugin watches the model with this entity name.
ROBOT_ENTITY = 'turtlebot3'


def _robot(context):
    model = LaunchConfiguration('model').perform(context)
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    # robot models in this package (e.g. burger_cam_bag) take precedence over turtlebot3_gazebo's
    sdf = os.path.join(get_package_share_directory('autorace_gazebo'), 'models', f'turtlebot3_{model}', 'model.sdf')
    if not os.path.exists(sdf):
        sdf = os.path.join(tb3_gazebo, 'models', f'turtlebot3_{model}', 'model.sdf')
    if not os.path.exists(sdf):
        raise RuntimeError(f'TurtleBot3 model not found: {sdf}')

    # URDF only drives TF; burger_cam has no separate URDF, so reuse burger's.
    urdf_model = 'burger' if model.startswith('burger') else model
    urdf_path = os.path.join(get_package_share_directory('turtlebot3_description'),
                             'urdf', f'turtlebot3_{urdf_model}.urdf')
    with open(urdf_path) as f:
        robot_description = f.read()
    if model == 'burger_cam_bag':
        # match the SDF: 70 mm wheels, 180 mm wheel separation
        robot_description = (robot_description
                             .replace('${namespace}', '')
                             .replace('xyz="0.0 0.08 0.023"', 'xyz="0.0 0.09 0.025"')
                             .replace('xyz="0.0 -0.080 0.023"', 'xyz="0.0 -0.09 0.025"'))
        # Keep sensor TF identical to the SDF that is actually spawned.
        robot = ET.fromstring(robot_description)
        mount = ET.parse(sdf).getroot().find('model/link[@name="camera_link"]')
        frames = [('camera_link', 'base_footprint', mount.findtext('pose'))]
        for sensor in mount.findall('sensor[@type="camera"]'):
            name = sensor.get('name')
            frames.append((name + '_sensor_frame', 'camera_link', sensor.findtext('pose')))
            frames.append((name + '_optical_frame', name + '_sensor_frame',
                           f'0 0 0 {-math.pi / 2} 0 {-math.pi / 2}'))
        for child, parent, pose in frames:
            xyzrpy = pose.split()
            ET.SubElement(robot, 'link', {'name': child})
            joint = ET.SubElement(robot, 'joint', {'name': child + '_joint', 'type': 'fixed'})
            ET.SubElement(joint, 'parent', {'link': parent})
            ET.SubElement(joint, 'child', {'link': child})
            ET.SubElement(joint, 'origin', {'xyz': ' '.join(xyzrpy[:3]), 'rpy': ' '.join(xyzrpy[3:])})
        robot_description = ET.tostring(robot, encoding='unicode')

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-entity', ROBOT_ENTITY,
                '-timeout', '120',
                '-file', sdf,
                '-x', LaunchConfiguration('x').perform(context),
                '-y', LaunchConfiguration('y').perform(context),
                '-z', LaunchConfiguration('z').perform(context),
                '-Y', LaunchConfiguration('yaw').perform(context),
            ],
        ),
    ]
    if model == 'burger_cam_bag':
        nodes.append(Node(
            package='autorace_gazebo', executable='camera_adapter.py',
            output='screen', parameters=[{'use_sim_time': True}],
        ))
    return nodes


def generate_launch_description():
    pkg = get_package_share_directory('autorace_gazebo')
    gazebo_ros = get_package_share_directory('gazebo_ros')
    tb3_models = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models')
    model_path = os.pathsep.join(filter(None, [
        os.path.join(pkg, 'models'), tb3_models, os.environ.get('GAZEBO_MODEL_PATH', '')]))
    plugin_path = os.pathsep.join(filter(None, [
        os.path.join(get_package_prefix('autorace_gazebo'), 'lib'), os.environ.get('GAZEBO_PLUGIN_PATH', '')]))

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(pkg, 'worlds', 'autorace_v014_full.world'),
                              description='autorace_v014_full.world (default) or autorace_v014.world (floor only)'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('referee', default_value='true', description='run race_manager.py'),
        DeclareLaunchArgument('race_config', default_value=os.path.join(pkg, 'config', 'race_runtime.yaml')),
        DeclareLaunchArgument('model', default_value='burger_cam_bag',
                              description='turtlebot3 model: burger_cam_bag (camera matched to the real robot/bag), '
                                          'burger_cam, burger, waffle, waffle_pi'),
        # Red start line at world (1.031, -1.999) on the map scaled by 260/227 (build_assets.py
        # MAP_SCALE); course yaw -3.14 is applied by the floor model.
        DeclareLaunchArgument('x', default_value='0.687'),
        DeclareLaunchArgument('y', default_value='-1.999'),
        DeclareLaunchArgument('z', default_value='0.010'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', plugin_path),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(gui),
        ),
        OpaqueFunction(function=_robot),
        Node(
            package='autorace_gazebo',
            executable='race_manager.py',
            name='race_manager',
            output='screen',
            parameters=[LaunchConfiguration('race_config')],
            condition=IfCondition(LaunchConfiguration('referee')),
        ),
    ])
