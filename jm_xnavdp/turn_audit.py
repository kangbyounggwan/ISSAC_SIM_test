"""Measured turn response and physical-clock parity; not goal-arrival success."""
import numpy as np


def audit_turn(rows, sign):
    turn=[r for r in rows if 2.5<=r['time_s']<=11.9]
    if len(turn)<2: return dict(pass_=False,reason='INSUFFICIENT_SAMPLES',**{'pass':False})
    times=np.asarray([r['time_s'] for r in turn])
    yaw=np.unwrap(np.radians([r['yaw_deg'] for r in turn]))
    positions=np.asarray([r['position_world'][:2] for r in turn])
    drift=float(np.linalg.norm(positions-positions[0],axis=1).max())
    signed_rate=float(sign*(yaw[-1]-yaw[0])/(times[-1]-times[0]))
    latest=float(np.mean([sign*r['yaw_rate_rps'] for r in turn if r['time_s']>=9.]))
    contact=float(max(r['contact_N'] for r in rows))
    tilt=float(max(r['tilt_deg'] for r in rows))
    clock_error=float(max(abs(r['actual_physics_elapsed_s']-r['time_s']) for r in rows))
    return dict(**{'pass':signed_rate>=.10 and latest>=.10 and drift<=.20 and contact<=.20 and tilt<15 and clock_error<.02},
                signed_mean_yaw_rate_rps=signed_rate,signed_late_yaw_rate_rps=latest,
                accumulated_yaw_deg=float(np.degrees(yaw[-1]-yaw[0])),max_turn_drift_m=drift,
                max_contact_N=contact,max_tilt_deg=tilt,max_physics_clock_error_s=clock_error,
                criteria='signed average and late yaw rate >=.10rad/s; drift <=.20m; contact <=.20N; tilt <15deg; clock error <.02s')
