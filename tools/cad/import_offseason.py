"""Convert the offseason Onshape export package to articulated AdvantageScope assets.

Run in a Python 3.14 environment with requirements.txt installed:
    python tools/cad/import_offseason.py '/path/to/offseason STEP'

STEP sources remain outside Git. --cache-dir can reuse intermediate OCCT GLBs.
All final vertices use robot coordinates (X forward, Y left, Z up), in meters.
"""

import argparse
import hashlib
import io
import json
from pathlib import Path
import struct
import tempfile

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh


ROBOT_FROM_CAD = np.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
TURRET_PIVOT = [-0.120650000, 0.203835000, 0.374650000]
# Center of the two hood pivot bearings; tiny STEP assembly skew is ignored.
HOOD_PIVOT = [-0.014985981, 0.203835000, 0.457770893]


def convert_step(source, output):
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.collections import (
        IndexedDataMap_TCollection_AsciiString_TCollection_AsciiString,
        Sequence_TDF_Label,
    )
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.Message import Message_ProgressRange
    from OCP.RWGltf import RWGltf_CafWriter
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TCollection import TCollection_AsciiString, TCollection_ExtendedString
    from OCP.TDocStd import TDocStd_Document
    from OCP.XCAFDoc import XCAFDoc_DocumentTool

    document = TDocStd_Document(TCollection_ExtendedString('offseason-cad'))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.SetColorMode(True)
    if reader.ReadFile(str(source)) != IFSelect_RetDone or not reader.Transfer(document):
        raise ValueError(f'Could not import {source}')
    tool = XCAFDoc_DocumentTool.ShapeTool_s(document.Main())
    roots = Sequence_TDF_Label()
    tool.GetFreeShapes(roots)
    if roots.Length() != 1:
        raise ValueError(f'Expected one assembled root in {source}')
    BRepMesh_IncrementalMesh(tool.GetShape_s(roots.Value(1)), 0.5, False, 0.35, True)
    writer = RWGltf_CafWriter(TCollection_AsciiString(str(output)), True)
    writer.SetMergeFaces(True)
    # OCCT imports STEP in millimeters; GLB vertices must be meters.
    writer.ChangeCoordinateSystemConverter().SetInputLengthUnit(0.001)
    if not writer.Perform(
        document,
        IndexedDataMap_TCollection_AsciiString_TCollection_AsciiString(),
        Message_ProgressRange(),
    ):
        raise ValueError(f'Could not export {output}')


def load_scene(path):
    data = path.read_bytes()
    length, kind = struct.unpack_from('<II', data, 12)
    header = json.loads(data[20:20 + length])
    # Instance indices remain consistent between the two endpoint exports. Unique
    # names prevent duplicate part names from acquiring random trimesh suffixes.
    for index, node in enumerate(header['nodes']):
        node['name'] = f"{node.get('name', 'node')}__{index}"
    materials = header.setdefault('materials', [])
    default_material = len(materials)
    materials.append({'pbrMetallicRoughness': {
        'baseColorFactor': [0.65, 0.65, 0.65, 1], 'metallicFactor': 0, 'roughnessFactor': 0.7,
    }})
    for mesh in header['meshes']:
        for primitive in mesh['primitives']:
            primitive.setdefault('material', default_material)
    chunk = json.dumps(header).encode()
    chunk += b' ' * (-len(chunk) % 4)
    tail = data[20 + length:]
    rebuilt = (struct.pack('<III', 0x46546C67, 2, 20 + len(chunk) + len(tail))
               + struct.pack('<II', len(chunk), kind) + chunk + tail)
    return trimesh.load(io.BytesIO(rebuilt), file_type='glb', merge_primitives=True)


def motion_groups(retracted, extended):
    if set(retracted.graph.nodes_geometry) != set(extended.graph.nodes_geometry):
        raise ValueError('Intake endpoint instance lists differ')
    groups = {}
    for node in sorted(retracted.graph.nodes_geometry):
        before, geometry_before = retracted.graph[node]
        after, geometry_after = extended.graph[node]
        # Identical B-reps can tessellate with slightly different vertex counts.
        # Local bounds should agree within the chosen meshing tolerance.
        if not np.allclose(retracted.geometry[geometry_before].bounds,
                           extended.geometry[geometry_after].bounds, atol=0.0005, rtol=0):
            raise ValueError(f'Intake geometry changed between endpoints: {node}')
        delta = after @ np.linalg.inv(before)
        key = tuple(np.round(delta.flatten(), 6))
        groups.setdefault(key, {'nodes': [], 'matrix': delta})['nodes'].append(node)
    groups = sorted(groups.values(), key=lambda group: -len(group['nodes']))
    if len(groups) != 3 or not np.allclose(groups[0]['matrix'][:3, :3], np.eye(3), atol=1e-8):
        raise ValueError('Expected a level carriage and two rotating parallel-link groups')
    # Upper arm's pivot is higher. Both groups must turn by the same angle.
    for group in groups[1:]:
        delta = group['matrix']
        pivot = np.linalg.pinv(np.eye(3) - delta[:3, :3], rcond=1e-8) @ delta[:3, 3]
        group['pivot'] = ROBOT_FROM_CAD[:3, :3] @ pivot
        group['angle_degrees'] = float(np.degrees(Rotation.from_matrix(delta[:3, :3]).as_rotvec()[0]))
    groups[1:] = sorted(groups[1:], key=lambda group: -group['pivot'][2])
    if not np.allclose(groups[1]['matrix'][:3, :3], groups[2]['matrix'][:3, :3], atol=1e-8):
        raise ValueError('Deploy links do not rotate together')
    rotation = groups[1]['matrix'][:3, :3]
    link_vector = np.linalg.pinv(rotation - np.eye(3), rcond=1e-8) @ groups[0]['matrix'][:3, 3]
    return groups, ROBOT_FROM_CAD[:3, :3] @ link_vector


def export_scene(source, output, nodes=None, color=None):
    result = trimesh.Scene()
    geometries = set()
    for node in sorted(nodes if nodes is not None else source.graph.nodes_geometry):
        transform, geometry = source.graph[node]
        if geometry not in geometries:
            mesh = source.geometry[geometry].copy()
            if color is not None:
                mesh.visual = trimesh.visual.TextureVisuals(material=trimesh.visual.material.PBRMaterial(
                    baseColorFactor=color, metallicFactor=0.1, roughnessFactor=0.65,
                ))
            result.add_geometry(mesh, node_name=node, geom_name=geometry,
                                transform=ROBOT_FROM_CAD @ transform)
            geometries.add(geometry)
        else:
            result.graph.update(frame_to=node, matrix=ROBOT_FROM_CAD @ transform, geometry=geometry)
    output.write_bytes(result.export(file_type='glb', include_normals=True))
    return {'file': output.name, 'instances': len(result.graph.nodes_geometry),
            'bounds_meters': result.bounds.tolist(), 'bytes': output.stat().st_size}


def component(pivot):
    return {'zeroedRotations': [], 'zeroedPosition': (-np.asarray(pivot)).tolist()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[2]
                        / 'advantagescope_assets/Robot_offseason')
    parser.add_argument('--cache-dir', type=Path)
    args = parser.parse_args()
    sources = ['base', 'turret', 'hood', 'intake-in', 'intake-out', 'full-reference']
    args.output.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='offseason-cad-') as temporary:
        cache = args.cache_dir or Path(temporary)
        cache.mkdir(parents=True, exist_ok=True)
        scenes, hashes = {}, {}
        for name in sources:
            source = args.source / f'offseason-{name}.step'
            digest = hashlib.sha256(source.read_bytes()).hexdigest()
            hashes[source.name] = digest
            intermediate = cache / f'{name}.glb'
            stamp = cache / f'{name}.sha256'
            if not intermediate.exists() or not stamp.exists() or stamp.read_text() != digest:
                print(f'Converting {source.name}', flush=True)
                convert_step(source, intermediate)
                stamp.write_text(digest)
            scenes[name] = load_scene(intermediate)
        groups, radius = motion_groups(scenes['intake-in'], scenes['intake-out'])
        assets = []
        for name, filename, color in [
            ('base', 'model.glb', None),
            ('hood', 'model_0.glb', [40, 220, 80, 255]),
            ('turret', 'model_1.glb', [40, 220, 80, 255]),
            ('hood', 'model_3.glb', [245, 205, 40, 255]),
            ('turret', 'model_4.glb', [45, 110, 235, 255]),
        ]:
            assets.append(export_scene(scenes[name], args.output / filename, color=color))
        for index, group in zip([2, 5, 6], groups):
            assets.append(export_scene(scenes['intake-in'], args.output / f'model_{index}.glb',
                                       nodes=group['nodes']))
        config = {
            '$schema': '../config-schema.json', 'name': '581 Offseason Bot',
            'rotations': [], 'position': [0, 0, 0], 'cameras': [],
            'components': [component(HOOD_PIVOT), component(TURRET_PIVOT), component([0, 0, 0]),
                           component(HOOD_PIVOT), component(TURRET_PIVOT),
                           component(groups[1]['pivot']), component(groups[2]['pivot'])],
        }
        (args.output / 'config.json').write_text(json.dumps(config, indent=2) + '\n')
        manifest = {
            'source_sha256': hashes, 'cad_to_robot_rotation_degrees_z': 90,
            'tessellation': {'linear_deflection_mm': 0.5, 'angular_deflection_radians': 0.35},
            'turret_pivot_meters': TURRET_PIVOT, 'hood_pivot_meters': HOOD_PIVOT,
            'deploy': {'angle_degrees': groups[1]['angle_degrees'],
                       'upper_pivot_meters': groups[1]['pivot'].tolist(),
                       'lower_pivot_meters': groups[2]['pivot'].tolist(),
                       'retracted_link_vector_meters': radius.tolist(),
                       'endpoint_translation_meters':
                           (ROBOT_FROM_CAD[:3, :3] @ groups[0]['matrix'][:3, 3]).tolist(),
                       'group_instance_counts': [len(g['nodes']) for g in groups],
                       'assumption': 'Link angle proportional to calibrated motor travel, 0 to 11.9 inches.'},
            'assets': sorted(assets, key=lambda asset: asset['file']),
        }
        (args.output / 'cad-manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        print(json.dumps(manifest['deploy'], indent=2))


if __name__ == '__main__':
    main()
