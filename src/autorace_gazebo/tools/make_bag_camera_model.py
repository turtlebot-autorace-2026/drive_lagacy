#!/usr/bin/env python3
"""Apply the measured/estimated camera rig to the existing robot SDF.

Only camera sensors are updated. Wheel dimensions, inertia, collision, drive,
LiDAR and all other original robot elements are retained. Run after editing
config/camera_rig.json. Positions are relative to the model origin, whose
nominal height above the ground is 10 mm.
"""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def element(parent, name, text):
    child = ET.SubElement(parent, name)
    child.text = str(text)
    return child


def apply(package):
    rig = json.loads((package / 'config/camera_rig.json').read_text())
    path = package / 'models/turtlebot3_burger_cam_bag/model.sdf'
    tree = ET.parse(path)
    link = tree.getroot().find('model/link[@name="camera_link"]')
    link_pose = [float(x) for x in link.findtext('pose').split()]
    if any(abs(x) > 1e-10 for x in link_pose[3:]):
        raise ValueError('camera_link must have zero orientation for this rig generator')
    for c in rig['cameras']:
        existing = link.find('sensor[@name="' + c['name'] + '"]')
        if existing is not None:
            link.remove(existing)
        sensor = ET.SubElement(link, 'sensor', {'name': c['name'], 'type': 'camera'})
        for key, val in [('always_on', 'true'), ('visualize', 'false'), ('update_rate', c['rate_hz'])]:
            element(sensor, key, val)
        pose = [v - origin for v, origin in zip(c['xyz_from_model_m'], link_pose[:3])]
        pose += [math.radians(v) for v in c['rpy_deg']]
        element(sensor, 'pose', ' '.join(f'{v:.10g}' for v in pose))
        camera = ET.SubElement(sensor, 'camera', {'name': c['name']})
        element(camera, 'horizontal_fov', 2 * math.atan(c['width'] / (2 * c['fx'])))
        im = ET.SubElement(camera, 'image')
        for key in ('width', 'height'):
            element(im, key, c[key])
        element(im, 'format', 'R8G8B8')
        clip = ET.SubElement(camera, 'clip')
        element(clip, 'near', .02)
        element(clip, 'far', 20)
        lens = ET.SubElement(camera, 'lens')
        intr = ET.SubElement(lens, 'intrinsics')
        for key in ('fx', 'fy', 'cx', 'cy'):
            element(intr, key, c[key])
        element(intr, 's', 0)
        plugin = ET.SubElement(sensor, 'plugin', {
            'name': c['name'] + '_driver', 'filename': 'libgazebo_ros_camera.so'})
        ros = ET.SubElement(plugin, 'ros')
        # Classic's ROS plugin assumes fx == fy for K. The adapter below emits
        # the actual rendered K instead; hide the plugin's approximate info.
        element(ros, 'remapping', c['name'] + '/camera_info:=/autorace/internal/' + c['name'] + '_info')
        element(ros, 'remapping', 'set_camera_info:=/' + c['name'] + '/set_camera_info')
        element(plugin, 'camera_name', c['name'])
        element(plugin, 'frame_name', c['name'] + '_optical_frame')
        for key in ('cx', 'cy'):
            element(plugin, key, c[key])
        for key in ('fx', 'fy', 'cx', 'cy'):
            element(plugin, 'P_' + key, c[key])
    ET.indent(tree, space='  ')
    # Humble spawn_entity parses a Unicode string with lxml, which rejects an
    # XML encoding declaration. SDF itself does not require the declaration.
    tree.write(path, encoding='unicode', xml_declaration=False)
    return rig


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package', type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    for c in apply(args.package)['cameras']:
        print(c['name'], 'xyz(model)', c['xyz_from_model_m'], 'rpy(deg)', c['rpy_deg'])
