#!/usr/bin/env python3
"""Generate Gazebo Classic assets for AutoRace v014 from the Blender handoff.

Outputs (inside this package):
  models/autorace_mission_devices_v014/   traffic light, barrier, two folding signs
  worlds/autorace_v014_full.world         floor + mission devices + Blender signs + props

Inputs are read only. Blender is not needed: visuals come from the GLB, frames,
joints and collision hulls from mission_devices.json. Static signs use the bundled
export_static_signs.py output from the .blend; walls/obstacles still use 2020 props.

usage: python3 -I tools/build_assets.py --handoff ~/autorace/simenv/autorace-v014-handoff
"""
import argparse
import json
import math
import os
import re
import shutil
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

PKG = Path(__file__).resolve().parents[1]
MODEL_NAME = 'autorace_mission_devices_v014'

# Blender frame node -> SDF link name. Visual/collision meshes go to the link of
# their nearest frame/joint ancestor.
LINKS = {
    'TrafficLight_ROOT_v012': 'traffic_light',
    'Barrier_ROOT_v012': 'barrier_base',
    'Barrier_Arm_Joint_v012': 'barrier_arm',
    'Intersection_Right_ROOT_v012': 'right_base',
    'Intersection_Right_Fold_Joint_v012': 'right_sign',
    'Intersection_Left_ROOT_v012': 'left_base',
    'Intersection_Left_Fold_Joint_v012': 'left_sign',
}
JOINTS = {
    'Barrier_Arm_Joint_v012': ('barrier_joint', 'barrier_base', 'barrier_arm'),
    'Intersection_Right_Fold_Joint_v012': ('right_sign_joint', 'right_base', 'right_sign'),
    'Intersection_Left_Fold_Joint_v012': ('left_sign_joint', 'left_base', 'left_sign'),
}
LENSES = {f'TrafficLight_Lens_{c}_v012': f'lens_{c}' for c in ('red', 'yellow', 'green')}

# Moving-link density for mass/inertia. Authoring assumption (plastic/aluminium mix).
DENSITY = 900.0

# glTF Y-up -> Blender Z-up: x_b = x_g, y_b = -z_g, z_b = y_g
G2B = np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]], float)

# Props from turtlebot3_autorace_2020.world kept at their original poses. The
# traffic light, barrier and right-turn sign are replaced by mission devices.
REPLACED_INCLUDES = {'course', 'traffic_bar', 'traffic_light', 'traffic_right', 'checker'}
SKIPPED_MODELS = {'ground2'}

# Horizontal map scale about the world origin. The handoff lane is 227 mm between the
# inner line edges (250 mm centre to centre, 273 mm outer); scaled to 260 mm inner.
# Floor, line widths, device/prop positions, wall lengths and judge regions scale;
# device/sign/obstacle sizes and all heights do not.
MAP_SCALE = 0.260 / 0.227
FLOOR_MODELS = ('autorace_panel_surface_flat', 'autorace_panel_surface_steps_2mm')
# Road plates are 30 mm tall in the handoff; flatten them to 0.5 mm so road and base
# are level (the 0.5 mm keeps the road texture above the base top without z-fighting).
ROAD_Z_SCALE = 0.0005 / 0.030

# Judge regions (world frame, metres, before MAP_SCALE). Authoring values, not regulation values.
# name: (center xyz, yaw, size xyz)
EXTRA_REGIONS = {
    # Red start line texture: x 0.886..0.915, lane between white y=-1.88 and yellow y=-1.61.
    'start_line': ((0.9005, -1.745, 0.18), 0.0, (0.03, 0.27, 0.30)),
    # Intersection island x 0.87..1.13, y -1.13..-0.38. Robot arrives heading -x.
    # Right branch = lane over the island top, left branch = lane under the island bottom.
    'branch_right': ((1.0, -0.255, 0.18), 0.0, (0.2, 0.22, 0.30)),
    'branch_left': ((1.0, -1.25, 0.18), 0.0, (0.2, 0.22, 0.30)),
}


# ---------------------------------------------------------------- math helpers

def mat(m):
    return np.array(m, float)


def rpy(R):
    """ZYX roll/pitch/yaw of a 3x3 rotation."""
    sy = -R[2, 0]
    pitch = math.asin(max(-1.0, min(1.0, sy)))
    if abs(sy) < 0.999999:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = 0.0
        yaw = math.atan2(-R[0, 1], R[1, 1])
    return roll, pitch, yaw


def pose_str(M):
    r, p, y = rpy(M[:3, :3])
    t = M[:3, 3]
    return ' '.join(f'{v:.6f}' for v in (*t, r, p, y))


def scale_xy(v):
    return (v[0] * MAP_SCALE, v[1] * MAP_SCALE, *v[2:])


def orthonormalize(M):
    """Strip scale from a 4x4 transform (frames in the JSON are rigid up to float noise)."""
    U, _, Vt = np.linalg.svd(M[:3, :3])
    out = M.copy()
    out[:3, :3] = U @ Vt
    return out


def srgb(c):
    c = max(0.0, min(1.0, float(c)))
    return 12.92 * c if c <= 0.0031308 else 1.055 * c ** (1 / 2.4) - 0.055


# ---------------------------------------------------------------- GLB reader

class Glb:
    COMP = {5126: ('f', 4), 5125: ('I', 4), 5123: ('H', 2), 5121: ('B', 1)}
    NCOMP = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4}

    def __init__(self, path):
        data = Path(path).read_bytes()
        jlen = struct.unpack_from('<I', data, 12)[0]
        self.j = json.loads(data[20:20 + jlen])
        off = 20 + jlen
        blen = struct.unpack_from('<I', data, off)[0]
        self.bin = data[off + 8:off + 8 + blen]

    def view(self, i):
        bv = self.j['bufferViews'][i]
        o = bv.get('byteOffset', 0)
        return self.bin[o:o + bv['byteLength']], bv.get('byteStride')

    def accessor(self, i):
        a = self.j['accessors'][i]
        raw, stride = self.view(a['bufferView'])
        fmt, size = self.COMP[a['componentType']]
        n = self.NCOMP[a['type']]
        start = a.get('byteOffset', 0)
        stride = stride or size * n
        out = np.empty((a['count'], n), float)
        for k in range(a['count']):
            out[k] = struct.unpack_from('<' + fmt * n, raw, start + k * stride)
        return out

    def image(self, i):
        im = self.j['images'][i]
        return self.view(im['bufferView'])[0], im.get('name', f'image{i}')

    def world_matrices(self):
        nodes = self.j['nodes']
        out = {}

        def local(n):
            if 'matrix' in n:
                return np.array(n['matrix'], float).reshape(4, 4).T
            T = np.eye(4)
            x, y, z, w = n.get('rotation', [0, 0, 0, 1])
            T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
            T[:3, :3] = T[:3, :3] @ np.diag(n.get('scale', [1, 1, 1]))
            T[:3, 3] = n.get('translation', [0, 0, 0])
            return T

        def walk(i, parent):
            M = parent @ local(nodes[i])
            out[nodes[i]['name']] = (i, M)
            for c in nodes[i].get('children', []):
                walk(c, M)

        for root in self.j['scenes'][self.j.get('scene', 0)]['nodes']:
            walk(root, np.eye(4))
        return out


# ---------------------------------------------------------------- COLLADA writer

class Dae:
    """Minimal COLLADA 1.4.1 (Z-up, metres) that Gazebo Classic's loader accepts."""

    def __init__(self):
        self.materials = {}   # name -> (rgba_srgb, texture file or None)
        self.parts = {}       # material name -> [P, N, UV, tri indices], merged
        self.geoms = []

    def add(self, mat_name, color, texture, P, N, UV, idx):
        # Merge everything sharing a material into one geometry: Gazebo Classic's
        # loader dropped untextured materials referenced by several geometries.
        self.materials.setdefault(mat_name, (color, texture))
        if mat_name not in self.parts:
            self.parts[mat_name] = [P, N, UV, np.asarray(idx)]
        else:
            p = self.parts[mat_name]
            base = len(p[0])
            p[0] = np.vstack([p[0], P])
            p[1] = np.vstack([p[1], N])
            p[2] = None if p[2] is None or UV is None else np.vstack([p[2], UV])
            p[3] = np.concatenate([p[3], np.asarray(idx) + base])
        self.geoms = [(f'g{i}', m, *v) for i, (m, v) in enumerate(self.parts.items())]

    def write(self, path):
        f = []
        w = f.append
        w('<?xml version="1.0" encoding="utf-8"?>')
        w('<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">')
        w('<asset><contributor><authoring_tool>autorace_gazebo build_assets.py</authoring_tool></contributor>'
          '<unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>')
        textures = sorted({t for _, t in self.materials.values() if t})
        w('<library_images>')
        for t in textures:
            w(f'<image id="{_id(t)}-img" name="{_id(t)}"><init_from>{t}</init_from></image>')
        w('</library_images><library_effects>')
        for m, (c, t) in self.materials.items():
            w(f'<effect id="{_id(m)}-fx"><profile_COMMON>')
            if t:
                w(f'<newparam sid="{_id(t)}-surf"><surface type="2D"><init_from>{_id(t)}-img</init_from></surface></newparam>'
                  f'<newparam sid="{_id(t)}-samp"><sampler2D><source>{_id(t)}-surf</source></sampler2D></newparam>')
            w('<technique sid="common"><phong>')
            w('<ambient><color>%s</color></ambient>' % ' '.join(f'{v:.4f}' for v in c))
            if t:
                w(f'<diffuse><texture texture="{_id(t)}-samp" texcoord="UVMap"/></diffuse>')
            else:
                w('<diffuse><color>%s</color></diffuse>' % ' '.join(f'{v:.4f}' for v in c))
            w('<specular><color>0.05 0.05 0.05 1</color></specular><emission><color>0 0 0 1</color></emission>'
              '<shininess><float>8</float></shininess>')
            w('</phong></technique></profile_COMMON></effect>')
        w('</library_effects><library_materials>')
        for m in self.materials:
            w(f'<material id="{_id(m)}" name="{_id(m)}"><instance_effect url="#{_id(m)}-fx"/></material>')
        w('</library_materials><library_geometries>')
        for gid, m, P, N, UV, idx in self.geoms:
            w(f'<geometry id="{gid}"><mesh>')
            _src(w, f'{gid}-p', P, 'XYZ')
            _src(w, f'{gid}-n', N, 'XYZ')
            if UV is not None:
                _src(w, f'{gid}-uv', UV, 'ST')
            w(f'<vertices id="{gid}-v"><input semantic="POSITION" source="#{gid}-p"/></vertices>')
            w(f'<triangles material="{_id(m)}" count="{len(idx) // 3}">'
              f'<input semantic="VERTEX" source="#{gid}-v" offset="0"/>'
              f'<input semantic="NORMAL" source="#{gid}-n" offset="1"/>')
            if UV is not None:
                w(f'<input semantic="TEXCOORD" source="#{gid}-uv" offset="2" set="0"/>')
            # One index per input: Gazebo Classic derives the stride from the input count.
            per = 2 if UV is None else 3
            w('<p>' + ' '.join(' '.join([str(int(i))] * per) for i in idx) + '</p></triangles></mesh></geometry>')
        w('</library_geometries><library_visual_scenes><visual_scene id="Scene" name="Scene">')
        for gid, m, *_ in self.geoms:
            w(f'<node id="{gid}-node" name="{gid}"><instance_geometry url="#{gid}"><bind_material><technique_common>'
              f'<instance_material symbol="{_id(m)}" target="#{_id(m)}">'
              f'<bind_vertex_input semantic="UVMap" input_semantic="TEXCOORD" input_set="0"/>'
              f'</instance_material></technique_common></bind_material></instance_geometry></node>')
        w('</visual_scene></library_visual_scenes><scene><instance_visual_scene url="#Scene"/></scene></COLLADA>')
        Path(path).write_text('\n'.join(f))


def _id(s):
    return re.sub(r'[^A-Za-z0-9_]', '_', s)


def _src(w, sid, A, params):
    n = len(params)
    w(f'<source id="{sid}"><float_array id="{sid}-a" count="{A.size}">'
      + ' '.join(f'{v:.6g}' for v in A.ravel())
      + f'</float_array><technique_common><accessor source="#{sid}-a" count="{len(A)}" stride="{n}">'
      + ''.join(f'<param name="{p}" type="float"/>' for p in params)
      + '</accessor></technique_common></source>')


# ---------------------------------------------------------------- mission devices

def link_of(name, nodes):
    while name is not None and name not in LINKS:
        name = nodes[name]['parent']
    return LINKS.get(name), name


def build_mission_devices(handoff):
    md = handoff / 'blender-map/assets/mission-devices-v014'
    spec = json.loads((md / 'mission_devices.json').read_text())
    nodes = {n['name']: n for n in spec['nodes']}
    glb = Glb(md / 'mission_devices_neutral.glb')
    gworld = glb.world_matrices()

    out = PKG / 'models' / MODEL_NAME
    if out.exists():
        shutil.rmtree(out)
    (out / 'meshes').mkdir(parents=True)

    frame_world = {f: orthonormalize(mat(nodes[f]['matrix_world_from_object_neutral'])) for f in LINKS}

    # Textures
    tex_files = {}
    for ti, tex in enumerate(glb.j.get('textures', [])):
        raw, name = glb.image(tex['source'])
        fn = f'{_id(name)}.png'
        (out / 'meshes' / fn).write_bytes(raw)
        tex_files[ti] = fn

    # Check GLB frames against the JSON (catches axis-conversion mistakes).
    max_err = 0.0
    for name, (_, Mg) in gworld.items():
        if name in nodes:
            max_err = max(max_err, float(np.abs(G2B @ Mg @ np.linalg.inv(G2B)
                                                - mat(nodes[name]['matrix_world_from_object_neutral'])).max()))
    if max_err > 1e-4:
        raise SystemExit(f'GLB/JSON frame mismatch {max_err:.2e}')

    visuals = {}  # (link, visual name) -> Dae
    for name, (gi, Mg) in gworld.items():
        gnode = glb.j['nodes'][gi]
        if 'mesh' not in gnode:
            continue
        link, frame = link_of(nodes[name]['parent'], nodes)
        if link is None:
            raise SystemExit(f'no link for {name}')
        vis = LENSES.get(name, f'{link}_visual')
        dae = visuals.setdefault((link, vis), Dae())
        # vertex (glTF local) -> link frame (Blender Z-up)
        T = np.linalg.inv(frame_world[frame]) @ G2B @ Mg
        Nrm = np.linalg.inv(T[:3, :3]).T
        for prim in glb.j['meshes'][gnode['mesh']]['primitives']:
            P = glb.accessor(prim['attributes']['POSITION'])
            P = (T[:3, :3] @ P.T).T + T[:3, 3]
            N = (Nrm @ glb.accessor(prim['attributes']['NORMAL']).T).T
            N /= np.linalg.norm(N, axis=1, keepdims=True) + 1e-12
            UV = None
            if 'TEXCOORD_0' in prim['attributes']:
                UV = glb.accessor(prim['attributes']['TEXCOORD_0'])
                UV[:, 1] = 1.0 - UV[:, 1]          # glTF top-left -> COLLADA bottom-left
            idx = glb.accessor(prim['indices']).ravel().astype(int)
            m = glb.j['materials'][prim['material']]
            pbr = m.get('pbrMetallicRoughness', {})
            base = pbr.get('baseColorFactor', [1, 1, 1, 1])
            color = [srgb(c) for c in base[:3]] + [1.0]
            tex = tex_files.get(pbr.get('baseColorTexture', {}).get('index'))
            if tex is None:
                UV = None
            dae.add(m['name'], color, tex, P, N, UV, idx)

    for (link, vis), dae in visuals.items():
        dae.write(out / 'meshes' / f'{vis}.dae')

    # Collisions: 8-vertex hulls become boxes; others become convex meshes.
    collisions = {}  # link -> [(name, xml)]
    volumes = {}
    for n in spec['nodes']:
        if n['role'] != 'collision':
            continue
        link, frame = link_of(n['parent'], nodes)
        T = np.linalg.inv(frame_world[frame]) @ mat(n['matrix_world_from_object_neutral'])
        V = np.array(n['vertices_local_m'], float)
        cname = _id(n['name'].replace('_v012__Collision', '')).lower() + '_collision'
        if len(V) == 8:
            # faces[0] = [0,3,2,1] bottom quad, vertex 4 is above vertex 0
            e = [V[1] - V[0], V[3] - V[0], V[4] - V[0]]
            size = [np.linalg.norm(x) for x in e]
            R = np.column_stack([x / s for x, s in zip(e, size)])
            if np.linalg.det(R) < 0:
                R[:, 1] *= -1
            B = np.eye(4)
            B[:3, :3] = R
            B[:3, 3] = V.mean(axis=0)
            M = T @ B
            geom = '<box><size>%s</size></box>' % ' '.join(f'{s:.6f}' for s in size)
            volumes.setdefault(link, []).append((size[0] * size[1] * size[2], M[:3, 3], size, M[:3, :3]))
        else:
            dae = Dae()
            P = (T[:3, :3] @ V.T).T + T[:3, 3]
            tri = []
            for face in n['faces']:
                for k in range(1, len(face) - 1):
                    tri += [face[0], face[k], face[k + 1]]
            dae.add('collision', [0.5, 0.5, 0.5, 1], None, P, np.tile([0, 0, 1.0], (len(P), 1)), None, tri)
            dae.write(out / 'meshes' / f'{cname}.dae')
            M = np.eye(4)
            geom = f'<mesh><uri>model://{MODEL_NAME}/meshes/{cname}.dae</uri></mesh>'
            ext = P.max(0) - P.min(0)
            volumes.setdefault(link, []).append((float(np.prod(ext)) * 0.6, (P.max(0) + P.min(0)) / 2, ext, np.eye(3)))
        collisions.setdefault(link, []).append(
            f'<collision name="{cname}"><pose>{pose_str(M)}</pose><geometry>{geom}</geometry>'
            '<surface><friction><ode><mu>0.6</mu><mu2>0.6</mu2></ode></friction>'
            '<contact><ode><kp>1e6</kp><kd>100</kd><max_vel>0.1</max_vel><min_depth>0.0005</min_depth></ode></contact>'
            '</surface></collision>')

    # SDF
    s = []
    w = s.append
    w('<?xml version="1.0"?>')
    w('<!-- Generated by tools/build_assets.py from the v014 Blender handoff. Do not edit by hand. -->')
    w('<sdf version="1.7">')
    w(f'<model name="{MODEL_NAME}">')
    w('<pose>0 0 0 0 0 0</pose>')
    for frame, link in LINKS.items():
        moving = frame in JOINTS
        F = frame_world[frame].copy()
        F[:2, 3] *= MAP_SCALE      # move the device with the scaled floor, keep its size
        w(f'<link name="{link}"><pose>{pose_str(F)}</pose>')
        if moving:
            w('<gravity>false</gravity>')
        w(_inertial(volumes.get(link, [])))
        for (l, vis) in visuals:
            if l == link:
                w(f'<visual name="{vis}"><geometry><mesh><uri>model://{MODEL_NAME}/meshes/{vis}.dae</uri></mesh></geometry></visual>')
        s.extend(collisions.get(link, []))
        w('</link>')
    for frame, link in LINKS.items():
        if frame not in JOINTS:
            w(f'<joint name="{link}_fixed" type="fixed"><parent>world</parent><child>{link}</child></joint>')
    for frame, (jname, parent, child) in JOINTS.items():
        n = nodes[frame]
        lo, hi = n['limit_rad']
        ax = ' '.join(str(v) for v in n['axis_local'])
        w(f'<joint name="{jname}" type="revolute"><parent>{parent}</parent><child>{child}</child>'
          f'<axis><xyz>{ax}</xyz><limit><lower>{lo:.9f}</lower><upper>{hi:.9f}</upper>'
          '<effort>5</effort><velocity>3.0</velocity></limit>'
          '<dynamics><damping>0.05</damping><friction>0</friction></dynamics></axis></joint>')
    w('</model></sdf>')
    (out / 'model.sdf').write_text('\n'.join(s))
    (out / 'model.config').write_text(
        '<?xml version="1.0"?>\n<model><name>AutoRace mission devices v014</name><version>1.0</version>'
        '<sdf version="1.7">model.sdf</sdf><description>Traffic light, barrier and folding intersection signs '
        'generated from the v014 Blender handoff (GLB visuals, JSON frames/joints/collision hulls).</description></model>\n')

    defaults = {JOINTS[f][0]: nodes[f]['default_q_rad'] for f in JOINTS}
    regions = {}
    for n in spec['nodes']:
        if n['role'] == 'trigger_reference':
            M = mat(n['matrix_world_from_object_neutral'])
            regions['s1' if '_S1_' in n['name'] else 's2'] = (
                scale_xy(tuple(M[:3, 3])), rpy(orthonormalize(M)[:3, :3])[2], scale_xy(tuple(n['full_size_m'])))
    print(f'mission devices: {len(visuals)} visuals, {sum(map(len, collisions.values()))} collisions, '
          f'GLB/JSON frame error {max_err:.1e}, defaults {defaults}')
    return defaults, regions


def _inertial(pieces):
    if not pieces:
        return '<inertial><mass>0.01</mass><inertia><ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz></inertia></inertial>'
    m_tot = sum(v for v, *_ in pieces) * DENSITY
    com = sum(v * np.asarray(c) for v, c, *_ in pieces) / sum(v for v, *_ in pieces)
    I = np.zeros((3, 3))
    for v, c, size, R in pieces:
        m = v * DENSITY
        a, b, d = size
        Ib = m / 12 * np.diag([b * b + d * d, a * a + d * d, a * a + b * b])
        r = np.asarray(c) - com
        I += R @ Ib @ R.T + m * (r @ r * np.eye(3) - np.outer(r, r))
    I += np.eye(3) * 1e-7
    return (f'<inertial><pose>{com[0]:.6f} {com[1]:.6f} {com[2]:.6f} 0 0 0</pose><mass>{max(m_tot, 1e-3):.6f}</mass>'
            f'<inertia><ixx>{I[0,0]:.3e}</ixx><ixy>{I[0,1]:.3e}</ixy><ixz>{I[0,2]:.3e}</ixz>'
            f'<iyy>{I[1,1]:.3e}</iyy><iyz>{I[1,2]:.3e}</iyz><izz>{I[2,2]:.3e}</izz></inertia></inertial>')


# ---------------------------------------------------------------- floor

def scale_floor(floor_model):
    """Apply MAP_SCALE (and ROAD_Z_SCALE to road plates) to a panel floor SDF in place.

    Idempotent: the old horizontal scale is divided out of the link poses.
    """
    path = floor_model / 'model.sdf'
    tree = ET.parse(path)
    for link in tree.getroot().find('model').findall('link'):
        meshes = link.findall('.//mesh')
        old = float(meshes[0].findtext('scale', '1 1 1').split()[0])
        pose = link.find('pose')
        v = [float(x) for x in pose.text.split()]
        v[0], v[1] = v[0] / old * MAP_SCALE, v[1] / old * MAP_SCALE
        pose.text = ' '.join(f'{x:.9g}' for x in v)
        for mesh in meshes:
            sc = mesh.find('scale')
            if sc is None:
                sc = ET.SubElement(mesh, 'scale')
            sz = ROAD_Z_SCALE if '__Road' in mesh.findtext('uri') else 1
            sc.text = f'{MAP_SCALE:.9g} {MAP_SCALE:.9g} {sz:.9g}'
    ET.indent(tree)
    tree.write(path, encoding='utf-8', xml_declaration=True)


class Floor:
    """Top height of the panel floor (z=0 base, road plate top where one exists)."""

    def __init__(self, floor_model):
        sdf = ET.parse(floor_model / 'model.sdf').getroot().find('model')
        yaw = float(sdf.findtext('pose').split()[5])
        c, s = math.cos(yaw), math.sin(yaw)
        self.R = np.array([[c, -s], [s, c]])
        self.tris = []
        for link in sdf.findall('link'):
            off = np.array([float(v) for v in link.findtext('pose').split()[:2]])
            for col in link.findall('collision'):
                uri = col.find('geometry/mesh/uri').text
                if '__Road' not in uri:
                    continue
                sc = np.array([float(v) for v in col.findtext('geometry/mesh/scale', '1 1 1').split()])
                P, idx = _read_dae(floor_model / uri)
                for k in range(0, len(idx), 3):
                    tri = P[idx[k:k + 3]]
                    if np.ptp(tri[:, 2]) < 1e-6 and tri[0, 2] > 0.02:     # road top faces only (unscaled)
                        self.tris.append((tri[:, :2] * sc[:2] + off, tri[0, 2] * sc[2]))

    def height(self, x, y):
        q = self.R.T @ np.array([x, y])
        for tri, z in self.tris:
            if _in_tri(q, tri):
                return z
        return 0.0


def _read_dae(path):
    txt = Path(path).read_text()
    P = np.array(re.search(r'<float_array id="positions-array"[^>]*>([^<]+)<', txt).group(1).split(), float).reshape(-1, 3)
    idx = []
    for p in re.findall(r'<triangles[^>]*>(.*?)</triangles>', txt, re.S):
        stride = 1 + max(int(o) for o in re.findall(r'offset="(\d+)"', p))
        raw = np.array(re.search(r'<p>([^<]+)</p>', p).group(1).split(), int)
        idx.extend(raw[::stride])
    return P, np.array(idx, int)


def _in_tri(p, t):
    d = [(p[0] - t[(i + 1) % 3][0]) * (t[i][1] - t[(i + 1) % 3][1])
         - (t[i][0] - t[(i + 1) % 3][0]) * (p[1] - t[(i + 1) % 3][1]) for i in range(3)]
    return not ((min(d) < -1e-9) and (max(d) > 1e-9))


# ---------------------------------------------------------------- world

def apply_static_signs(world):
    """Use the authored 3D signs instead of the legacy all-face textured boxes."""
    manifest = json.loads((PKG / 'config/static_signs_v014.json').read_text())
    if abs(manifest['map_position_scale'] - MAP_SCALE) > 1e-9:
        raise ValueError('Map scale changed: re-export static signs from Blender')
    found = set()
    for include in world.findall('include'):
        name = include.findtext('name') or include.findtext('uri').rsplit('/', 1)[-1]
        if name not in manifest['signs']:
            continue
        spec = manifest['signs'][name]
        if not (PKG / 'models' / spec['model'] / 'model.sdf').exists():
            raise ValueError('Missing exported sign: ' + spec['model'])
        include.find('uri').text = 'model://' + spec['model']
        include.find('pose').text = spec['pose']
        found.add(name)
    if found != set(manifest['signs']):
        raise ValueError('Missing static sign includes: ' + str(set(manifest['signs']) - found))


def build_world(defaults, regions):
    from ament_index_python.packages import get_package_share_directory
    tb3 = Path(get_package_share_directory('turtlebot3_gazebo'))
    src = ET.parse(tb3 / 'worlds/turtlebot3_autorace_2020.world').getroot().find('world')
    floor = Floor(PKG / 'models/autorace_panel_surface_flat')

    state = {m.get('name'): m.findtext('pose') for m in src.findall('state/model')}
    props = []
    lift_log = []
    for inc in src.findall('include'):
        uri = inc.findtext('uri').strip()
        name = uri.rsplit('/', 1)[-1]
        if name in REPLACED_INCLUDES:
            continue
        x, y, z, r, p, yaw = scale_xy([float(v) for v in inc.findtext('pose').split()])
        h = floor.height(x, y)
        lift_log.append((name, h))
        props.append(f'<include><uri>{uri}</uri><name>{name}</name>'
                     f'<pose>{x} {y} {z + h:.4f} {r} {p} {yaw}</pose></include>')
    for m in src.findall('model'):
        name = m.get('name')
        if name in SKIPPED_MODELS:
            continue
        m = ET.fromstring(ET.tostring(m))
        pose_el = m.find('pose')
        pose_el.attrib.pop('frame', None)
        if name in state:
            pose_el.text = state[name]
        mx, my, mz, *rest = scale_xy([float(v) for v in pose_el.text.split()])
        myaw = rest[2]
        h = 0.0
        for l in m.findall('link'):
            lp = l.find('pose')
            lx, ly = 0.0, 0.0
            if lp is not None:
                lp.attrib.pop('frame', None)
                lv = scale_xy([float(v) for v in lp.text.split()])
                lx, ly = lv[:2]
                lp.text = ' '.join(f'{v:.6g}' for v in lv)
            if l.get('name', '').startswith('Wall_'):
                # walls bound the road: stretch along their length, keep thickness/height
                for size in l.iter('size'):
                    sv = [float(v) for v in size.text.split()]
                    size.text = f'{sv[0] * MAP_SCALE:.6g} ' + ' '.join(f'{v:.6g}' for v in sv[1:])
            wx = mx + math.cos(myaw) * lx - math.sin(myaw) * ly
            wy = my + math.sin(myaw) * lx + math.cos(myaw) * ly
            h = max(h, floor.height(wx, wy))
        for el in m.iter():
            el.attrib.pop('frame', None)
        pose_el.text = f'{mx} {my} {mz + h:.4f} ' + ' '.join(str(v) for v in rest)
        lift_log.append((name, h))
        props.append(ET.tostring(m, encoding='unicode'))

    all_regions = dict(regions)
    all_regions.update({n: (scale_xy(c), yaw, scale_xy(s)) for n, (c, yaw, s) in EXTRA_REGIONS.items()})
    reg_xml = ''.join(
        f'<region name="{n}"><pose>{c[0]:.6f} {c[1]:.6f} {c[2]:.6f} 0 0 {yaw:.6f}</pose>'
        f'<size>{s[0]:.4f} {s[1]:.4f} {s[2]:.4f}</size></region>'
        for n, (c, yaw, s) in all_regions.items())
    lights = []
    for l in src.findall('light'):
        if l.get('name') == 'sun':
            continue
        lp = l.find('pose')
        if lp is not None:
            lp.text = ' '.join(f'{v:.6g}' for v in scale_xy([float(v) for v in lp.text.split()]))
        lights.append(ET.tostring(l, encoding='unicode'))
    lights = ''.join(lights)

    world = f'''<?xml version="1.0"?>
<!-- Generated by tools/build_assets.py. AutoRace v014: panel floor, mission devices,
     and the static props of turtlebot3_autorace_2020 (the map the Blender scene was rebuilt from).
     Static signs use the v014 .blend export. Other static props come from the 2020
     models: Blender-side wall, tunnel roof and obstacle edits are not included. -->
<sdf version="1.7">
  <world name="autorace_v014">
    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <ode>
        <solver><type>quick</type><iters>150</iters><precon_iters>0</precon_iters><sor>1.4</sor>
          <use_dynamic_moi_rescaling>1</use_dynamic_moi_rescaling></solver>
        <constraints><cfm>0.00001</cfm><erp>0.2</erp><contact_max_correcting_vel>2000.0</contact_max_correcting_vel>
          <contact_surface_layer>0.01</contact_surface_layer></constraints>
      </ode>
    </physics>
    <scene>
      <ambient>0.8 0.8 0.8 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>false</shadows>
    </scene>
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>0 -4.5 5.0 0 0.85 1.5708</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>
    <light name="sun" type="directional">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.3 0.2 -1.0</direction>
    </light>
    {lights}
    <model name="ground_plane">
      <static>true</static>
      <pose>0 0 -0.04 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry></collision>
        <visual name="visual"><cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material>
        </visual>
      </link>
    </model>
    <include><uri>model://autorace_panel_surface_flat</uri></include>
    <include>
      <uri>model://{MODEL_NAME}</uri>
      <plugin name="mission_devices" filename="libautorace_mission_devices_plugin.so">
        <ros><namespace>/autorace/devices</namespace></ros>
        <joint name="barrier_joint" default="{defaults['barrier_joint']:.9f}" speed="1.5708"/>
        <joint name="right_sign_joint" default="{defaults['right_sign_joint']:.9f}" speed="1.5708"/>
        <joint name="left_sign_joint" default="{defaults['left_sign_joint']:.9f}" speed="1.5708"/>
        <light_link>traffic_light</light_link>
        <default_light>red</default_light>
      </plugin>
    </include>
    {''.join(props)}
    <plugin name="race_regions" filename="libautorace_regions_plugin.so">
      <ros><namespace>/autorace/truth</namespace></ros>
      <update_rate>200</update_rate>
      <robot_model>turtlebot3</robot_model>
      {reg_xml}
    </plugin>
  </world>
</sdf>
'''
    # Pretty-print through ElementTree to validate XML.
    root = ET.fromstring(world.split('\n', 1)[1])
    apply_static_signs(root.find('world'))
    ET.indent(root)
    text = '<?xml version="1.0"?>\n' + ET.tostring(root, encoding='unicode').replace(
        '<sdf version="1.7">', '<!-- Generated by tools/build_assets.py. Do not edit by hand. -->\n<sdf version="1.7">', 1)
    (PKG / 'worlds/autorace_v014_full.world').write_text(text + '\n')
    print('props (name, floor lift m):', lift_log)
    print('regions:', {k: (tuple(round(v, 4) for v in c), round(y, 4), s) for k, (c, y, s) in all_regions.items()})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--handoff', type=Path, default=Path.home() / 'autorace/simenv/autorace-v014-handoff')
    a = ap.parse_args()
    for name in FLOOR_MODELS:
        scale_floor(PKG / 'models' / name)
    defaults, regions = build_mission_devices(a.handoff.expanduser())
    build_world(defaults, regions)
    # The floor-only Blender export omits normals and textured ambient reflectance.
    # Repair the Classic rendering data after every full asset regeneration.
    from improve_visuals import prepare
    prepare(PKG)


if __name__ == '__main__':
    main()
