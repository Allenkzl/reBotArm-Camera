#!/usr/bin/env python3
"""Offline FK sampling for in-place hold candidates. Never imports motor control."""
from __future__ import annotations
import argparse
import itertools
import json
from pathlib import Path
import numpy as np

if __package__:
    from .design_poses import load_robot_modules, safety_metrics
else:
    from design_poses import load_robot_modules, safety_metrics

LOWER = np.array([-55., -8., 1.03, -70., -40., -70.])
UPPER = np.array([100.]*6)


def available_radius(reference_z_m, table_z_m, mounting_uncertainty_m, reserve_m=.05):
    """Conservative sphere radius budget, only for the explicitly supplied geometry."""
    values = (reference_z_m, table_z_m, mounting_uncertainty_m, reserve_m)
    if not all(np.isfinite(v) for v in values) or mounting_uncertainty_m < 0 or reserve_m < 0:
        raise ValueError('finite geometry and nonnegative uncertainty/reserve required')
    return reference_z_m-table_z_m-mounting_uncertainty_m-reserve_m


def scan_pose(q_deg, evaluate, radius_deg=5.):
    q = np.asarray(q_deg, dtype=float)
    if q.shape != (6,) or not np.all(np.isfinite(q)):
        raise ValueError('six finite angles required')
    if not np.isfinite(radius_deg) or radius_deg < 4:
        raise ValueError('scan radius must cover at least +/-4 degrees')
    if np.any(q-radius_deg <= LOWER) or np.any(q+radius_deg >= UPPER):
        return {'q_deg': q.tolist(), 'pass': False, 'reason': 'joint margin', 'evaluations': 0}
    camera, frame = float('inf'), float('inf')
    mount_reference = None
    worst_camera, worst_frame = None, None
    count = 0
    # Explicit +/-4 box plus +/-5 grid including center/face/interior planes.
    offsets = set(itertools.product((-radius_deg, 0., radius_deg), repeat=6))
    offsets.update(itertools.product((-4., 4.), repeat=6))
    for delta in sorted(offsets):
        pose = q+np.array(delta)
        result = evaluate(np.deg2rad(pose))
        c, f = float(result['camera_z']), float(result['lowest_z'])
        if 'mount_reference_z' in result:
            z = float(result['mount_reference_z'])
            if not np.isfinite(z):
                raise ValueError('nonfinite mounting reference')
            mount_reference = z if mount_reference is None else min(mount_reference, z)
        if not np.isfinite(c) or not np.isfinite(f):
            raise ValueError('nonfinite FK result')
        count += 1
        if c < camera: camera, worst_camera = c, pose.tolist()
        if f < frame: frame, worst_frame = f, pose.tolist()
    return {'q_deg': q.tolist(), 'pass': camera >= .20 and frame >= .10,
            'reason': 'reference heights' if camera < .20 or frame < .10 else 'sampled checks passed',
            'radius_deg': radius_deg, 'evaluations': count,
            'min_camera_reference_z_m': camera, 'min_link_frame_z_m': frame,
            'min_link6_origin_z_m': mount_reference, 'motion_ready': False,
            'worst_camera_q_deg': worst_camera, 'worst_frame_q_deg': worst_frame}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-root', type=Path, default=Path('/home/com/reBotArm_control_py'))
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    pin, load_model, get_frame, pad = load_robot_modules(args.robot_root)
    model = load_model()
    frame = get_frame(model)
    def evaluate(q):
        q_model = pad(model, q, controlled_joints=6)
        result = safety_metrics(pin, model, q_model, frame, np.array([.075, 0., -.025]))
        data = model.createData()
        pin.forwardKinematics(model, data, q_model)
        pin.updateFramePlacements(model, data)
        link6 = model.getFrameId('link6')
        if link6 >= len(model.frames):
            raise ValueError('model lacks link6 reference')
        result['mount_reference_z'] = float(data.oMf[link6].translation[2])
        return result
    # Candidate reference orientations only, not commands or approved motions.
    candidates = [[0., j2, j3, 0., 0., 0.] for j2 in (0., 10., 20.) for j3 in (10., 15., 20.)]
    results = [scan_pose(q, evaluate) for q in candidates]
    report = {'offline_only': True, 'no_motion_authorized': True,
              'camera_reference_offset_m': [.075, 0., -.025],
              'j3_assumed_hard_min_deg': 1.03,
              'reference_warning': 'link6 origin is not a verified physical flange center; gripper_end has a fixed 0.16621m offset from link6 in the current URDF.',
              'limits': 'software bounds plus historical J3 hard minimum; other physical limits uncalibrated',
              'limitations': ['Sampled FK reference points, not continuous collision checking.',
                              'Camera body/lens/support geometry, table/obstacles, cables not modeled.',
                              'No approach or Home path approved; in-place hold has no automatic transition.',
                              'No reliable kp or payload identification follows from this scan alone.'],
              'results': results}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(report, ensure_ascii=False))

if __name__ == '__main__': main()
