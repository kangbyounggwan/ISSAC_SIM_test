"""Read-only footstep diagnostics. Thresholds are fixed before evaluation.

An ankle body-origin rise is a lift proxy, not exact sole/ground clearance.
Contact is the net force on the ankle rigid body, not a pressure map.
"""
import argparse
import json
import math
from pathlib import Path
from statistics import mean

THRESHOLDS = dict(unloaded_force_n=5., landing_force_n=20., min_air_s=.08,
                  min_lift_m=.02, min_landing_s=.08)


def wrap_deg(x):
    return (x + 180.) % 360. - 180.


def norm_xy(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def angle_travel(rows, key, foot=None):
    values = [r[key] if foot is None else r[key][foot] for r in rows]
    return sum(wrap_deg(b-a) for a,b in zip(values, values[1:]))


def foot_events(rows, foot, start=2., end=14.):
    dt = rows[1]['time_s'] - rows[0]['time_s']
    events = []
    i = 1
    while i < len(rows):
        if not (start <= rows[i]['time_s'] < end and rows[i]['foot_force_n'][foot] < 5.
                and rows[i-1]['foot_force_n'][foot] >= 5.):
            i += 1
            continue
        begin = i
        while i < len(rows) and rows[i]['time_s'] < end and rows[i]['foot_force_n'][foot] < 5.:
            i += 1
        finish = i
        duration = (finish-begin)*dt
        if duration + 1e-9 < .08:
            continue
        # Require a loaded sample before takeoff; an initially airborne foot is not a step.
        before = begin-1
        while before > 0 and rows[before]['foot_force_n'][foot] < 20.:
            before -= 1
        had_support = rows[before]['foot_force_n'][foot] >= 20.
        base = rows[before]['foot_position_world'][foot]
        rise = max(r['foot_position_world'][foot][2] for r in rows[begin:finish]) - base[2]
        landing = None
        dwell = max(1, math.ceil(.08/dt-1e-8))
        # A brief 5..20N transition may occur before a stable landing.
        for j in range(finish, min(len(rows)-dwell+1, finish+int(.5/dt)+1)):
            if all(r['foot_force_n'][foot] >= 20. for r in rows[j:j+dwell]):
                landing = j
                break
        qualified = had_support and rise >= .02 and landing is not None
        events.append(dict(takeoff_s=rows[begin]['time_s'], air_duration_s=duration,
            peak_ankle_rise_m=rise, prior_support=had_support,
            stable_landing_s=rows[landing]['time_s'] if landing is not None else None,
            landing_xy_displacement_m=norm_xy(rows[landing]['foot_position_world'][foot],base) if landing is not None else None,
            landing_yaw_change_deg=wrap_deg(rows[landing]['foot_yaw_deg'][foot]-rows[before]['foot_yaw_deg'][foot]) if landing is not None else None,
            verified_lift_landing=qualified))
    return events


def summarize(rows, manifest):
    active = [r for r in rows if 2. <= r['time_s'] <= 14.]
    plateau = [r for r in rows if 4. <= r['time_s'] < 14.]
    brake = [r for r in rows if r['time_s'] >= 17.]
    if len(active)<2 or not plateau:
        return dict(evaluable=False, reason='Terminated before a full turn interval', duration_s=rows[-1]['time_s'])
    feet=[]
    for k,name in enumerate(manifest['feet']):
        events=foot_events(rows,k)
        feet.append(dict(name=name,unloaded_intervals=events,
            verified_lift_landing_count=sum(e['verified_lift_landing'] for e in events),
            min_force_n=min(r['foot_force_n'][k] for r in active),
            max_force_n=max(r['foot_force_n'][k] for r in active),
            plateau_contact_fraction=mean(r['foot_force_n'][k]>=20. for r in plateau),
            turn_foot_yaw_travel_deg=angle_travel(active,'foot_yaw_deg',k),
            turn_net_xy_displacement_m=norm_xy(active[-1]['foot_position_world'][k],active[0]['foot_position_world'][k]),
            plateau_max_xy_from_4s_m=max(norm_xy(r['foot_position_world'][k],plateau[0]['foot_position_world'][k]) for r in plateau),
            plateau_ankle_z_range_m=max(r['foot_position_world'][k][2] for r in plateau)-min(r['foot_position_world'][k][2] for r in plateau),
            plateau_mean_speed_xy_mps=mean(math.hypot(*r['foot_velocity_world'][k][:2]) for r in plateau)))
    joint_names=manifest['joint_order']
    selected=['left_hip_yaw_joint','right_hip_yaw_joint','torso_joint','left_knee_joint','right_knee_joint']
    joints={}
    for name in selected:
        j=joint_names.index(name)
        joints[name]=dict(at_2s_deg=math.degrees(active[0]['joint_position'][j]),
            at_14s_deg=math.degrees(active[-1]['joint_position'][j]),
            final_deg=math.degrees(rows[-1]['joint_position'][j]),
            plateau_range_deg=math.degrees(max(r['joint_position'][j] for r in plateau)-min(r['joint_position'][j] for r in plateau)))
    return dict(evaluable=True,thresholds=THRESHOLDS,feet=feet,joints=joints,
        duration_s=rows[-1]['time_s'],
        pelvis_turn_2_to_14_deg=angle_travel(active,'pelvis_yaw_world_deg'),
        torso_turn_2_to_14_deg=angle_travel(active,'torso_yaw_world_deg'),
        plateau_pelvis_turn_4_to_14_deg=angle_travel(plateau,'pelvis_yaw_world_deg'),
        plateau_both_feet_contact_fraction=mean(all(f>=20. for f in r['foot_force_n']) for r in plateau),
        plateau_command_yaw_mean_rps=mean(r['command_applied'][2] for r in plateau),
        plateau_abs_yaw_rate_mean_rps=mean(abs(r['yaw_rate_rps']) for r in plateau),
        plateau_action_range_max=max(max(r['action_applied'][j] for r in plateau)-min(r['action_applied'][j] for r in plateau) for j in range(19)),
        final_yaw_deg=rows[-1]['yaw_deg'],final_yaw_error_deg=rows[-1]['yaw_error_deg'],
        max_drift_m=max(r['drift_m'] for r in rows),
        final_both_feet_contact_fraction=mean(all(f>=20. for f in r['foot_force_n']) for r in brake) if brake else None,
        limitations=['50Hz snapshots can miss brief contacts','ankle rise is not exact sole clearance',
            'contact net force is not a pressure map','single fixed start and seed; no JM rack/carry validation'])


def main():
    p=argparse.ArgumentParser()
    p.add_argument('root',type=Path)
    args=p.parse_args()
    manifest=json.loads((args.root/'manifest.json').read_text(encoding='utf-8'))
    reports={}
    for file in sorted(args.root.glob('*/telemetry.json')):
        rows=json.loads(file.read_text(encoding='utf-8'))
        report=summarize(rows,manifest)
        reports[file.parent.name]=report
        (file.parent/'footstep_audit.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    (args.root/'footstep_summary.json').write_text(json.dumps(reports,indent=2),encoding='utf-8')
    print(json.dumps(reports,indent=2))


if __name__=='__main__':main()
