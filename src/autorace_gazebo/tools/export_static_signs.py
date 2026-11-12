#!/usr/bin/env python3
"""Export the seven visible static signs from the original v014 .blend.

Run with Blender 5 in background mode; the input file is never saved or changed:
  blender -b --factory-startup --disable-autoexec --python tools/export_static_signs.py -- \
      --blend /path/to/autorace_mcp_v014_mission_devices.blend

Archived rectangular props and hidden collision references are excluded. Evaluated
bevels, split normals, UVs, front/back material assignments and packed image bytes
come from the .blend. The bundled DAE models are sufficient at simulation runtime.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import bpy
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_assets import Dae, Floor, MAP_SCALE, pose_str, srgb

SIGNS = ('traffic_construction', 'traffic_intersection', 'traffic_noentry',
         'traffic_parking', 'traffic_pl_left', 'traffic_stop', 'traffic_tunnel')


def material_info(material, directory):
    bsdf = next(n for n in material.node_tree.nodes if n.type == 'BSDF_PRINCIPLED')
    base = bsdf.inputs['Base Color']
    texture = None
    if base.is_linked:
        node = base.links[0].from_node
        if node.type != 'TEX_IMAGE' or node.image is None:
            raise ValueError('Unsupported base-color network: ' + material.name)
        img = node.image
        if img.packed_file is None:
            raise ValueError('Expected packed source texture: ' + img.name)
        texture = Path(img.name).name
        (directory / texture).write_bytes(bytes(img.packed_file.data))
        color = [1.0, 1.0, 1.0, 1.0]
    else:
        color = [srgb(c) for c in base.default_value[:3]] + [1.0]
    return color, texture


def export(blend, package):
    bpy.ops.wm.open_mainfile(filepath=str(blend), load_ui=False, use_scripts=False)
    depsgraph = bpy.context.evaluated_depsgraph_get()
    floor = Floor(package / 'models/autorace_panel_surface_flat')
    manifest = {'source_blend': blend.name,
                'source_sha256': hashlib.sha256(blend.read_bytes()).hexdigest(),
                'blender_version': bpy.app.version_string,
                'map_position_scale': MAP_SCALE, 'sign_geometry_scale': 1.0,
                'collision_policy': 'Static triangle mesh of visible plate, post, socket and base',
                'signs': {}}
    for name in SIGNS:
        # Support_* is a panel attachment, not the sign's original model frame.
        # The authored plate retains the sign pose (including its intended yaw).
        frame = bpy.data.objects[name + '__Plate_v006'].matrix_world.copy()
        objects = sorted((o for o in bpy.context.scene.objects
                          if o.name.startswith(name + '__') and o.type == 'MESH'
                          and not o.hide_render and o.visible_get()), key=lambda o: o.name)
        if len(objects) != (11 if name == 'traffic_noentry' else 10):
            raise ValueError('Unexpected visible parts: ' + repr([o.name for o in objects]))
        model_name = 'autorace_' + name + '_v014'
        directory = package / 'models' / model_name
        meshes = directory / 'meshes'
        meshes.mkdir(parents=True, exist_ok=True)
        visual, collision = Dae(), Dae()
        all_positions, parts, face_normals = [], [], []
        for obj in objects:
            evaluated = obj.evaluated_get(depsgraph)
            mesh = evaluated.to_mesh(preserve_all_data_layers=True, depsgraph=depsgraph)
            try:
                mesh.calc_loop_triangles()
                transform = np.array(frame.inverted() @ evaluated.matrix_world)
                ntransform = np.linalg.inv(transform[:3, :3]).T
                positions = np.array([v.co[:] for v in mesh.vertices])
                positions = positions @ transform[:3, :3].T + transform[:3, 3]
                all_positions.append(positions)
                major = any(key in obj.name for key in ('__Plate_', '__RearPlate_', '__Post_', '__Socket_', '__Base_'))
                for index, material in enumerate(mesh.materials):
                    triangles = [t for t in mesh.loop_triangles if t.material_index == index]
                    if not triangles:
                        continue
                    color, texture = material_info(material, meshes)
                    vi = np.array([v for t in triangles for v in t.vertices])
                    li = [loop for t in triangles for loop in t.loops]
                    p = positions[vi]
                    normals = np.array([mesh.corner_normals[loop].vector[:] for loop in li]) @ ntransform.T
                    normals /= np.linalg.norm(normals, axis=1, keepdims=True)
                    uv = np.array([mesh.uv_layers.active.data[loop].uv[:] for loop in li]) if texture else None
                    indices = np.arange(len(p))
                    visual.add(material.name, color, texture, p, normals, uv, indices)
                    if major:
                        collision.add('collision', [.5, .5, .5, 1], None, p, normals, None, indices)
                    if texture and '__Plate_' in obj.name:
                        tri = p.reshape(-1, 3, 3)
                        face_normals.append(np.cross(tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0]).sum(axis=0))
                parts.append({'name': obj.name, 'vertices_evaluated': len(mesh.vertices),
                              'triangles_evaluated': len(mesh.loop_triangles), 'collision': major})
            finally:
                evaluated.to_mesh_clear()
        visual.write(meshes / 'visual.dae')
        collision.write(meshes / 'collision.dae')
        (directory / 'model.config').write_text(
            '<?xml version="1.0"?><model><name>' + model_name + '</name><version>1.0</version>'
            '<sdf version="1.7">model.sdf</sdf><description>Static sign from the v014 Blender reference.</description></model>\n')
        (directory / 'model.sdf').write_text(
            '<?xml version="1.0"?>\n<sdf version="1.7"><model name="' + model_name + '">'
            '<static>true</static><link name="sign">'
            '<visual name="visual"><geometry><mesh><uri>model://' + model_name + '/meshes/visual.dae</uri></mesh></geometry></visual>'
            '<collision name="collision"><geometry><mesh><uri>model://' + model_name + '/meshes/collision.dae</uri></mesh></geometry></collision>'
            '</link></model></sdf>\n')
        world = np.array(frame)
        world[:2, 3] *= MAP_SCALE
        vertices = np.vstack(all_positions)
        # The simulator already flattens 30 mm Blender road panels to 0.5 mm.
        # Seat each base on that existing floor without scaling sign dimensions.
        ground = floor.height(*world[:2, 3])
        source_origin_z = float(world[2, 3])
        world[2, 3] = ground - vertices[:, 2].min()
        front = np.sum(face_normals, axis=0)
        front /= np.linalg.norm(front)
        manifest['signs'][name] = {'model': model_name, 'pose': pose_str(world),
                                  'source_plate_matrix_world': [list(row) for row in frame],
                                  'source_origin_z_m': source_origin_z, 'sim_ground_z_m': ground,
                                  'local_bounds_m': [vertices.min(0).tolist(), vertices.max(0).tolist()],
                                  'front_normal_local': front.tolist(), 'parts': parts,
                                  'textures': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in meshes.glob('*.png')}}
    target = package / 'config/static_signs_v014.json'
    target.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + '\n')
    print('EXPORTED_STATIC_SIGNS', len(manifest['signs']), str(target), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--blend', required=True, type=Path)
    parser.add_argument('--package', type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args(sys.argv[sys.argv.index('--') + 1:])
    export(args.blend.resolve(), args.package.resolve())
