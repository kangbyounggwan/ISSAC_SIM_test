"""Fixed, feedback-based JM walk/left-turn/walk experiment (no IK or RL training)."""
import math


def wrap(x):
    return (x + math.pi) % (2 * math.pi) - math.pi


class WalkTurnSequence:
    PHASES = ('SETTLE', 'MOVE_A', 'BRAKE_A', 'LEFT_TURN', 'BRAKE_TURN', 'MOVE_B', 'FINAL_STOP', 'DONE')
    LIMITS = dict(position_m=.12, stopped_position_m=.18, heading_deg=7.,
                  speed_mps=.10, yaw_rate_rps=.15, stable_s=1.,
                  tilt_deg=15., turn_abort_drift_m=.35, in_place_audit_m=.20)
    TIMEOUTS = dict(SETTLE=6., MOVE_A=12., BRAKE_A=6., LEFT_TURN=12.,
                    BRAKE_TURN=6., MOVE_B=12., FINAL_STOP=6.)

    def __init__(self, spawn):
        self.spawn = list(spawn)
        self.a = [spawn[0], spawn[1]+1.]
        self.b = [spawn[0]-1., spawn[1]+1.]
        self.phase = 'SETTLE'
        self.entered = 0.
        self.stable_since = None
        self.events = []
        self.failure = None
        self.turn_origin = None
        self.turn_max_drift = 0.
        self.command = [0., 0., 0.]
        self.last_t = None

    def target(self):
        if self.phase in ('MOVE_A', 'BRAKE_A', 'LEFT_TURN', 'BRAKE_TURN'):
            return self.a, math.pi if self.phase in ('LEFT_TURN', 'BRAKE_TURN') else math.pi/2
        if self.phase in ('MOVE_B', 'FINAL_STOP', 'DONE'):
            return self.b, math.pi
        return self.spawn, math.pi/2

    def _next(self, t, xy):
        old = self.phase
        self.phase = self.PHASES[self.PHASES.index(old)+1]
        self.events.append(dict(time_s=float(t), before=old, after=self.phase,
                                position_xy=[float(v) for v in xy]))
        self.entered = t
        self.stable_since = None
        if self.phase == 'LEFT_TURN':
            self.turn_origin = list(xy)

    def update(self, t, xy, yaw, speed, yaw_rate, tilt):
        values = [t, *xy, yaw, speed, yaw_rate, tilt]
        if not all(math.isfinite(v) for v in values):
            self.failure = 'NONFINITE_STATE'
        if self.last_t is not None and t < self.last_t:
            self.failure = 'CLOCK_REVERSED'
        if self.failure or self.phase == 'DONE':
            return [0., 0., 0.]
        if tilt >= self.LIMITS['tilt_deg']:
            self.failure = 'POSTURE_ABORT'
            return [0., 0., 0.]
        if t-self.entered > self.TIMEOUTS[self.phase]:
            self.failure = self.phase+'_TIMEOUT'
            return [0., 0., 0.]
        target, heading = self.target()
        dx, dy = target[0]-xy[0], target[1]-xy[1]
        distance = math.hypot(dx,dy)
        error = wrap(heading-yaw)
        target_cmd = [0.,0.,0.]
        stable = speed < .10 and abs(yaw_rate) < .15 and tilt < 15.
        if self.phase in ('MOVE_A', 'MOVE_B'):
            forward = math.cos(yaw)*dx+math.sin(yaw)*dy
            lateral = -math.sin(yaw)*dx+math.cos(yaw)*dy
            target_cmd = [max(-.12,min(.30,.8*forward)), max(-.12,min(.12,.8*lateral)), max(-.20,min(.20,.8*error))]
            if distance < .12 and abs(error) < math.radians(7):
                self._next(t,xy)
                target_cmd = [0.,0.,0.]
        elif self.phase == 'LEFT_TURN':
            self.turn_max_drift = max(self.turn_max_drift, math.dist(xy,self.turn_origin))
            if self.turn_max_drift > .35:
                self.failure = 'TURN_DRIFT_ABORT'
                return [0.,0.,0.]
            target_cmd[2] = max(-.30,min(.30,.8*error))
            if abs(error) < math.radians(7):
                self._next(t,xy)
                target_cmd = [0.,0.,0.]
        else:
            if self.phase == 'SETTLE':
                stable = stable and t >= 1.
            elif self.phase in ('BRAKE_A','FINAL_STOP'):
                stable = stable and distance < .18 and abs(error) < math.radians(7)
            elif self.phase == 'BRAKE_TURN':
                stable = stable and abs(error) < math.radians(7)
                self.turn_max_drift = max(self.turn_max_drift, math.dist(xy,self.turn_origin))
                if self.turn_max_drift > .35:
                    self.failure = 'TURN_DRIFT_ABORT'
                    return [0.,0.,0.]
            if stable:
                if self.stable_since is None:
                    self.stable_since = t
                if t-self.stable_since >= 1.:
                    self._next(t,xy)
            else:
                self.stable_since = None
        dt = .005 if self.last_t is None else t-self.last_t
        self.last_t = t
        # Velocity-command slew limits, not manually selected joint angles.
        for i, acceleration in enumerate((.4,.4,.6)):
            delta = max(-acceleration*dt,min(acceleration*dt,target_cmd[i]-self.command[i]))
            self.command[i] += delta
        return self.command.copy()


class ContinuousWalkTurnSequence(WalkTurnSequence):
    """Separate motion test: never claims the strict stop experiment passed.

    Same targets, policy command bounds, motion gates and safety limits. Omits
    certified intermediate stops; zero-command final 2s is an audit, not a gate.
    """
    PHASES=('WARMUP','MOVE_A','LEFT_TURN','MOVE_B','STOP_AUDIT','DONE')
    TIMEOUTS=dict(WARMUP=3.,MOVE_A=12.,LEFT_TURN=12.,MOVE_B=12.,STOP_AUDIT=3.)

    def __init__(self,spawn):
        super().__init__(spawn)
        self.phase='WARMUP'

    def target(self):
        if self.phase=='STOP_AUDIT': return self.b,math.pi
        return super().target()

    def update(self,t,xy,yaw,speed,yaw_rate,tilt):
        if self.phase not in ('WARMUP','STOP_AUDIT'):
            return super().update(t,xy,yaw,speed,yaw_rate,tilt)
        if not all(math.isfinite(v) for v in (t,*xy,yaw,speed,yaw_rate,tilt)):
            self.failure='NONFINITE_STATE'
        if self.last_t is not None and t<self.last_t: self.failure='CLOCK_REVERSED'
        if tilt>=15.: self.failure='POSTURE_ABORT'
        if self.failure: return [0.,0.,0.]
        dt=.005 if self.last_t is None else t-self.last_t
        self.last_t=t
        for i,acceleration in enumerate((.4,.4,.6)):
            self.command[i]+=max(-acceleration*dt,min(acceleration*dt,-self.command[i]))
        if t-self.entered>=2.:
            self._next(t,xy)
        return self.command.copy()
