#!/usr/bin/env python3
"""Repair the Blender floor's Classic lighting data without changing geometry.

The 4K reference texture is kept byte for byte. This is an idempotent export
postprocess: add face normals, supply the textured material's missing ambient
reflectance, and use a neutral key/fill rig instead of imported point lights.
"""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


NS = 'http://www.collada.org/2005/11/COLLADASchema'
ET.register_namespace('', NS)


def tag(name):
    return '{' + NS + '}' + name


def value(parent, name, text):
    child = parent.find(name)
    if child is None:
        child = ET.SubElement(parent, name)
    child.text = text
    return child


def repair_mesh(path):
    tree = ET.parse(path)
    root = tree.getroot()
    changed = False
    added = 0
    for phong in root.iter(tag('phong')):
        diffuse = phong.find(tag('diffuse'))
        if diffuse is not None and diffuse.find(tag('texture')) is not None:
            ambient = phong.find(tag('ambient'))
            if ambient is None:
                ambient = ET.Element(tag('ambient'))
                phong.insert(0, ambient)
            color = ambient.find(tag('color'))
            if color is None:
                color = ET.SubElement(ambient, tag('color'))
            if color.text != '1 1 1 1':
                color.text = '1 1 1 1'
                changed = True

    for mesh in root.iter(tag('mesh')):
        normals = []
        pending = []
        for triangles in mesh.findall(tag('triangles')):
            inputs = triangles.findall(tag('input'))
            if any(i.get('semantic') == 'NORMAL' for i in inputs):
                continue
            vertex_input = next(i for i in inputs if i.get('semantic') == 'VERTEX')
            vertex = mesh.find(tag('vertices') + '[@id="' + vertex_input.get('source')[1:] + '"]')
            position_input = next(i for i in vertex if i.get('semantic') == 'POSITION')
            source = mesh.find(tag('source') + '[@id="' + position_input.get('source')[1:] + '"]')
            positions = np.fromstring(source.find(tag('float_array')).text, sep=' ').reshape(-1, 3)
            stride = 1 + max(int(i.get('offset', '0')) for i in inputs)
            indices = np.fromstring(triangles.find(tag('p')).text, sep=' ', dtype=int).reshape(-1, 3, stride)
            vertices = positions[indices[:, :, int(vertex_input.get('offset', '0'))]]
            face_normals = np.cross(vertices[:, 1] - vertices[:, 0], vertices[:, 2] - vertices[:, 0])
            lengths = np.linalg.norm(face_normals, axis=1)
            valid = lengths > 1e-12
            face_normals[valid] /= lengths[valid, None]
            face_normals[~valid] = (0, 0, 1)
            first = len(normals)
            normals.extend(face_normals)
            normal_indices = np.repeat(np.arange(first, first + len(indices))[:, None, None], 3, axis=1)
            pending.append((triangles, np.concatenate([indices, normal_indices], axis=2), stride))

        if not normals:
            continue
        normal_id = mesh.find(tag('vertices')).get('id') + '-face-normals'
        source = ET.Element(tag('source'), {'id': normal_id})
        ET.SubElement(source, tag('float_array'), {
            'id': normal_id + '-array', 'count': str(len(normals) * 3),
        }).text = ' '.join(format(float(v), '.9g') for row in normals for v in row)
        common = ET.SubElement(source, tag('technique_common'))
        accessor = ET.SubElement(common, tag('accessor'), {
            'source': '#' + normal_id + '-array', 'count': str(len(normals)), 'stride': '3',
        })
        for axis in 'XYZ':
            ET.SubElement(accessor, tag('param'), {'name': axis, 'type': 'float'})
        mesh.insert(list(mesh).index(mesh.find(tag('vertices'))), source)
        for triangles, indices, offset in pending:
            normal_input = ET.Element(tag('input'), {
                'semantic': 'NORMAL', 'source': '#' + normal_id, 'offset': str(offset),
            })
            triangles.insert(len(triangles.findall(tag('input'))), normal_input)
            triangles.find(tag('p')).text = ' '.join(str(int(i)) for i in indices.ravel())
        changed = True
        added += len(normals)

    if changed:
        ET.indent(tree, space='  ')
        tree.write(path, encoding='utf-8', xml_declaration=True)
    return {'changed': changed, 'face_normals_added': added}


def repair_world(path):
    tree = ET.parse(path)
    world = tree.getroot().find('world')
    scene = world.find('scene')
    value(scene, 'ambient', '0.45 0.45 0.45 1')
    value(scene, 'background', '0.18 0.20 0.23 1')
    value(scene, 'shadows', 'false')
    for light in list(world.findall('light')):
        if light.get('name', '').startswith('user_point_light_'):
            world.remove(light)
    sun = world.find('light[@name="sun"]')
    value(sun, 'diffuse', '0.55 0.55 0.55 1')
    value(sun, 'specular', '0.05 0.05 0.05 1')
    fill = world.find('light[@name="soft_fill"]')
    if fill is None:
        fill = ET.Element('light', {'name': 'soft_fill', 'type': 'directional'})
        world.insert(list(world).index(sun) + 1, fill)
    for name, text in {
        'pose': '0 0 8 0 0 0', 'diffuse': '0.12 0.12 0.12 1',
        'specular': '0 0 0 1', 'direction': '0.4 -0.3 -1', 'cast_shadows': 'false',
    }.items():
        value(fill, name, text)
    ground = world.find('model[@name="ground_plane"]/link/visual/material')
    ground.clear()
    for name, text in {
        'ambient': '0.18 0.20 0.23 1', 'diffuse': '0.35 0.38 0.42 1',
        'specular': '0 0 0 1', 'emissive': '0 0 0 1',
    }.items():
        value(ground, name, text)
    ET.indent(tree, space='  ')
    tree.write(path, encoding='utf-8', xml_declaration=True)


def prepare(package):
    report = {}
    for name in ('autorace_panel_surface_flat', 'autorace_panel_surface_steps_2mm'):
        for path in sorted((package / 'models' / name / 'meshes').glob('*.dae')):
            report[str(path.relative_to(package))] = repair_mesh(path)
    for path in sorted((package / 'worlds').glob('*.world')):
        repair_world(path)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package', type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    print(json.dumps(prepare(args.package), indent=2))
