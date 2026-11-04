"""Hard-gated candidate selection and forward-only SIMULATION trial follower.

This is a test controller, not a claim of H1 safety certification. It never
changes the requested goal, gait weights, joint angles or success tolerances.
"""
import math
import numpy as np


def truncate_goal_plane(path, goal):
    """Trim predicted overshoot at the fixed goal's plane; preserve the raw output."""
    path = np.asarray(path, dtype=float)[:, :2]
    goal = np.asarray(goal, dtype=float)[:2]
    length = np.linalg.norm(goal)
    if length < 1e-5: return np.zeros((1, 2))
    axis = goal / length
    out = [np.zeros(2)]
    for point in path:
        if point @ axis >= length:
            prev = out[-1]
            alpha = (length-prev@axis) / max((point-prev)@axis, 1e-9)
            out.append(prev+np.clip(alpha, 0, 1)*(point-prev))
            break
        out.append(point)
    return np.asarray(out)


def depth_points(depth, intrinsic, base_height, stride=8):
    """Level-yaw camera, body offset (.12,0,.55), pitch down 8 degrees.

    Only observed obstacle surfaces, not a free-space/occlusion certification.
    """
    dep = np.asarray(depth)[::stride, ::stride]
    v, u = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    k = np.asarray(intrinsic)
    valid = np.isfinite(dep) & (dep >= .1) & (dep <= 5.)
    z = dep[valid]
    xr = (u[valid]-k[0,2])*z/k[0,0]
    yd = (v[valid]-k[1,2])*z/k[1,1]
    p = math.radians(8)
    x = math.cos(p)*z-math.sin(p)*yd+.12
    y = -xr
    h = base_height+.55-math.sin(p)*z-math.cos(p)*yd
    mask = (h > .15) & (h < 1.75)
    return np.c_[x[mask], y[mask]]


def densify(path, spacing=.04):
    if len(path)<2: return np.asarray(path)
    return np.concatenate([np.linspace(a,b,max(2,int(np.ceil(np.linalg.norm(b-a)/spacing))+1))
                           for a,b in zip(path[:-1],path[1:])])


def prefix_by_length(path, length):
    if not np.isfinite(length) or length<=0: raise ValueError('Invalid committed horizon')
    out=[path[0]]; traveled=0.
    for a,b in zip(path[:-1],path[1:]):
        d=float(np.linalg.norm(b-a))
        if traveled+d>=length:
            out.append(a+(b-a)*(length-traveled)/max(d,1e-12)); break
        out.append(b); traveled+=d
    return np.asarray(out)


def select_path(candidates, values, goal, obstacles_xy, radius=.38, commit_horizon=None):
    candidates, values = np.asarray(candidates), np.asarray(values)
    if candidates.ndim!=3 or candidates.shape[1:]!=(24,3) or values.shape!=(len(candidates),):
        raise ValueError('Invalid candidate contract')
    checks=[]; eligible=[]
    for i, raw in enumerate(candidates):
        reason=[]; margin=None
        if not np.isfinite(raw).all() or not np.isfinite(values[i]):
            checks.append({'id':i,'reject':['NONFINITE']}); continue
        path=truncate_goal_plane(raw,goal)
        full_path=path.copy()
        if commit_horizon is not None: path=prefix_by_length(path,commit_horizon)
        if len(path)<2 or np.linalg.norm(path[-1])<min(.20,np.linalg.norm(goal[:2])*.5):
            reason.append('INSUFFICIENT_PROGRESS')
        if np.any(path[:,0]<-.02) or np.any(np.diff(path[:,0])<-.02):
            reason.append('REVERSE')
        if len(obstacles_xy):
            dense=densify(path)
            margin=float(np.sqrt(((dense[:,None,:]-obstacles_xy[None,:,:])**2).sum(-1)).min() - radius)
            if margin<0: reason.append('OBSERVED_SURFACE_CLEARANCE')
        lengths=np.linalg.norm(np.diff(path,axis=0),axis=1)
        arc=np.r_[0.,np.cumsum(lengths)]
        target=path[min(int(np.searchsorted(arc,.45)),len(path)-1)]
        heading=math.atan2(target[1],target[0])
        if abs(heading)>math.radians(60): reason.append('HEADING')
        checks.append({'id':i,'q':float(values[i]),'reject':reason,'observed_margin_m':margin,
                       'raw_endpoint':raw[-1,:2].tolist(),'trimmed_endpoint':path[-1].tolist(),
                       'goal_clipped_endpoint':full_path[-1].tolist(),'committed_horizon_m':commit_horizon})
        if not reason: eligible.append((float(values[i]),i,path))
    if not eligible: return {'pass':False,'reason':'NO_ADMISSIBLE_MODEL_PATH','candidates':checks}
    _,i,path=max(eligible,key=lambda row:row[0])
    return {'pass':True,'selected_id':i,'path_body_xy':path.tolist(),'candidates':checks,
            'selection':'highest model Q after hard constraints; no synthetic detour fallback'}


def follow(path_world, position, yaw, goal_xy, max_speed=.4):
    """Forward/yaw proposal. Zero lateral, no root or joint writes."""
    path=np.asarray(path_world); position=np.asarray(position)[:2]
    closest=int(np.argmin(np.linalg.norm(path-position,axis=1)))
    tail=np.vstack([position,path[closest:]])
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(tail,axis=0),axis=1))]
    point=tail[min(int(np.searchsorted(arc,.45)),len(tail)-1)]
    delta=point-position
    err=math.atan2(math.sin(math.atan2(delta[1],delta[0])-yaw),math.cos(math.atan2(delta[1],delta[0])-yaw))
    distance=float(np.linalg.norm(np.asarray(goal_xy)-position))
    return np.array([min(max_speed,.65*distance)*max(0.,math.cos(err))**2,0.,np.clip(1.6*err,-.45,.45)])
