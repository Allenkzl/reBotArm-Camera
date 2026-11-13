"""Compare saved framing telemetry with a timestamped candump -L capture."""
import argparse
import json
import re
from collections import Counter, defaultdict
from pathlib import Path
from statistics import median


def decode(line):
    match = re.match(r'^\((\d+\.\d+)\)\s+\S+\s+([0-9A-Fa-f]{8})#([0-9A-Fa-f]{16})\s*$', line)
    if not match:
        return None
    timestamp, can_id, payload = match.groups()
    can_id = int(can_id, 16)
    kind = (can_id >> 24) & 31
    extra = (can_id >> 8) & 65535
    address = can_id & 255
    joint = address if kind == 1 else extra & 255
    if kind not in (1, 2) or joint not in (2, 3):
        return None
    position, velocity, third, fourth = (int(payload[i:i+4], 16) for i in range(0, 16, 4))
    fields = {
        'angle_deg': (position / 32767 - 1) * 720,
        'velocity_rad_s': (velocity / 32767 - 1) * 50,
    }
    if kind == 1:
        fields.update(kp=third * 5000 / 65535, kd=fourth * 100 / 65535,
                      feedforward_nm=(extra / 32767 - 1) * 36)
    else:
        fields.update(estimated_torque_nm=(third / 32767 - 1) * 36,
                      fault_bits=(extra >> 8) & 63, mode_state=extra >> 14)
    return float(timestamp), joint, kind, fields


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('log', type=Path)
    args = parser.parse_args()
    phases = defaultdict(list)
    with args.log.open() as source:
        for line in source:
            row = json.loads(line)
            if row['kind'] == 'photo_hold_sample':
                phases[row['phase']].append(row)
    with args.capture.open() as source:
        frames = [frame for line in source if (frame := decode(line)) is not None]
    counts = Counter((joint, kind) for _, joint, kind, _ in frames)
    if not phases or not all(counts[joint, kind] for joint in (2, 3) for kind in (1, 2)):
        parser.error('framing samples and both CAN directions for J2 and J3 are required')
    for phase, samples in phases.items():
        if phase.endswith('_ramp'):
            continue
        end = samples[-1]['t_wall']
        start = max(end-2, samples[0]['t_wall'])
        window = [row for row in samples if start <= row['t_wall'] <= end]
        for joint in (2, 3):
            result = {'phase': phase, 'joint': joint, 'software_samples': len(window)}
            for key in ('q_target_deg', 'q_actual_deg', 'tau_ff_cmd_nm', 'kp_nominal'):
                result[key] = median(row[key][joint-1] for row in window)
            for kind, label in ((1, 'command'), (2, 'feedback')):
                selected = [fields for t, j, k, fields in frames if j == joint and k == kind and start <= t <= end]
                if not selected:
                    parser.error(f'no timestamp match for {phase}, J{joint}, {label}')
                result[label] = {'frames': len(selected)}
                for key in selected[0]:
                    values = [f[key] for f in selected]
                    result[label][key] = sorted(set(values)) if key in ('fault_bits', 'mode_state') else median(values)
            print(json.dumps(result, ensure_ascii=False))


if __name__ == '__main__':
    main()
