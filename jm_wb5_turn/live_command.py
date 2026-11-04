"""Bounded velocity commands for a local simulator, never hardware control."""
import math

TTL_S = .5
MAX_SPEED = .18
MAX_REVERSE = .10
MAX_YAW = .25


def decode(payload, session, now):
    stopped = dict(enabled=False, velocity=[0., 0., 0.], reason='NO_COMMAND', view='third', sequence=0)
    if not isinstance(payload, dict) or payload.get('session') != session:
        return stopped
    timestamp = payload.get('at')
    if not isinstance(timestamp, (int, float)) or not math.isfinite(timestamp) or not 0 <= now-timestamp <= TTL_S:
        return dict(stopped, reason='INPUT_EXPIRED')
    view = payload.get('view', 'third')
    if view not in ('third', 'rgb', 'depth', 'lidar'):
        return dict(stopped, reason='INVALID_VIEW')
    stopped.update(view=view, sequence=payload.get('sequence', 0))
    if payload.get('enabled') is not True:
        return dict(stopped, reason='DISARMED')
    v = payload.get('velocity')
    if not isinstance(v, list) or len(v) != 3 or any(type(x) not in (int, float) or not math.isfinite(x) for x in v):
        return dict(stopped, reason='INVALID_COMMAND')
    if not -MAX_REVERSE <= v[0] <= MAX_SPEED or v[1] != 0 or abs(v[2]) > MAX_YAW:
        return dict(stopped, reason='OUT_OF_RANGE')
    return dict(enabled=True, velocity=v, view=view, sequence=payload.get('sequence', 0), reason='COMMAND_VALID')


def slew(current, target, dt=.02):
    return [a+max(-limit*dt, min(limit*dt, b-a)) for a, b, limit in zip(current, target, (.4, .4, .6))]
