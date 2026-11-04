"""Navigation-only rack handoff: logical assumed grasp, no object attachment."""
import math
import numpy as np
from keti_navigation.planner import astar, lookahead_target


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class RackTransfer:
    def __init__(self, pickup, unload, yaw, obstacles, turn_rate=.30, departure_turn='shortest', backoff=0., mode='forward'):
        if not 0 < turn_rate <= .45:
            raise ValueError('Outside existing yaw command envelope')
        self.turn_rate = turn_rate
        if departure_turn not in ('shortest','positive'):
            raise ValueError('Invalid departure turn')
        self.departure_turn=departure_turn
        self.departure_aligned=False
        if backoff not in (0., .25): raise ValueError('Use predefined backoff trial')
        self.backoff=backoff
        self.failure=None
        if mode not in ('forward','strafe'): raise ValueError('Unknown transfer mode')
        self.mode=mode
        self.pickup = np.array(pickup, dtype=float)
        self.unload = np.array(unload, dtype=float)
        self.yaw = yaw
        self.obstacles = obstacles
        self.state = 'PICKUP_APPROACH'
        self.assumed_grasp = False
        self.events = []
        self.path = None
        self.progress = 0.
        self.dock_latched = False

    def pickup_complete(self, position, time_s):
        if self.state != 'PICKUP_APPROACH':
            raise RuntimeError('Duplicate or invalid pickup handoff')
        self.state = 'ASSUMED_GRASP'
        self.assumed_grasp = True
        self.hold_until = time_s + 1.
        self.events.append(dict(time_s=time_s, event='ASSUMED_GRASP', physical_grasp=False,
                                position_world=np.asarray(position,dtype=float).tolist()))

    def start_transfer(self, position, time_s):
        if self.state not in ('ASSUMED_GRASP','RETREAT_FROM_RACK') or time_s < self.hold_until:
            return False
        if self.backoff:
            if self.state=='ASSUMED_GRASP':
                self.state='RETREAT_FROM_RACK'
                self.retreat_start=np.asarray(position,dtype=float)[:2].copy()
                self.retreat_time=time_s
                self.events.append(dict(time_s=time_s,event=self.state,distance_m=self.backoff,
                    experimental_reverse=True,position_world=np.asarray(position,dtype=float).tolist()))
            forward=np.array([math.cos(self.yaw),math.sin(self.yaw)])
            progress=float((self.retreat_start-np.asarray(position)[:2])@forward)
            if progress<self.backoff:
                if time_s-self.retreat_time>10.: self.failure='BACKOFF_TIMEOUT'
                return False
        # Current measured pose, not a snapped nominal pickup pose.
        self.route = astar(position[:2], self.unload, self.obstacles,
                           [34., 15.5, 40.5, 19.], .05, .38)
        self.path = np.asarray(self.route['path_xy'])
        self.state = 'ASTAR_TRANSFER'
        self.events.append(dict(time_s=time_s, event='ASTAR_TRANSFER',
                                map_source='authored USD collider AABBs including parked robots',
                                route=self.route))
        return True

    def command(self, position, yaw):
        if self.state=='RETREAT_FROM_RACK':
            return np.array([-.12,0.,np.clip(1.6*wrap(self.yaw-yaw),-.3,.3)]), self.state
        if self.path is None:
            return np.zeros(3), 'ASSUMED_GRASP'
        p = np.asarray(position)[:2]
        distance = float(np.linalg.norm(self.unload-p))
        if distance < .12:
            self.dock_latched = True
        elif distance >= .18:
            # Alignment is not permanent: leave it when measured position drifts
            # outside the unchanged mission success radius. Reuse safe path following.
            self.dock_latched = False
        if self.dock_latched:
            # Do not relax arrival tolerance or invent reverse gait to recover.
            return np.array([0., 0., np.clip(1.6*wrap(self.yaw-yaw), -.3, .3)]), 'UNLOAD_ALIGN_STOP'
        target, self.progress = lookahead_target(p, self.path, self.progress, .30)
        delta = target-p
        if self.mode=='strafe':
            c,s=math.cos(yaw),math.sin(yaw)
            local=np.array([c*delta[0]+s*delta[1],-s*delta[0]+c*delta[1]])
            local*=min(.18,.65*distance)/max(np.linalg.norm(local),1e-9)
            return np.array([np.clip(local[0],-.12,.20),np.clip(local[1],-.15,.15),
                             np.clip(1.6*wrap(self.yaw-yaw),-.3,.3)]),'ASTAR_STRAFE'
        heading = wrap(math.atan2(delta[1], delta[0])-yaw)
        if not self.departure_aligned:
            if abs(heading)<=math.radians(25): self.departure_aligned=True
            elif self.departure_turn=='positive':
                # Explicit alternate departure direction, no pose or joint edits.
                return np.array([0.,0.,self.turn_rate]), 'ASTAR_DEPARTURE_TURN_POSITIVE'
        speed = min(.30, .65*distance) * max(0., math.cos(heading))**2
        if abs(heading) > math.radians(25):
            speed = 0.
        return np.array([speed, 0., np.clip(1.6*heading, -self.turn_rate, self.turn_rate)]), 'ASTAR_TRANSFER'

    def finish(self, position, time_s):
        if self.state != 'ASTAR_TRANSFER':
            raise RuntimeError('Unload without transfer')
        self.state = 'UNLOAD_POSITION_REACHED'
        self.events.append(dict(time_s=time_s, event=self.state, position_world=np.asarray(position,dtype=float).tolist(),
                                actual_release=False))
