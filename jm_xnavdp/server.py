"""Loopback-only RGB-D planner service. Returns proposals, never drives H1."""
import argparse
import io
import hashlib
import json
from pathlib import Path
import sys
import threading
import time
import zipfile
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import numpy as np
from flask import Flask, request, jsonify
from jm_xnavdp.runtime import Runtime
from jm_xnavdp.adapter import local_goal, propose_command, world_path
from jm_xnavdp.physical_planner import depth_points, select_path, prefix_by_length, densify
from jm_xnavdp.depth_route import DepthRoute


def decode_packet(raw):
    required = {'rgb', 'depth_m', 'base_world', 'yaw_rad', 'goal_world_xy', 'intrinsic', 'captured_unix_s'}
    with zipfile.ZipFile(io.BytesIO(raw)) as z:
        names = [i.filename for i in z.infolist()]
        if set(names) != {k + '.npy' for k in required} or len(names) != len(required):
            raise ValueError('Unexpected packet fields')
        if sum(i.file_size for i in z.infolist()) > 16 * 1024 * 1024:
            raise ValueError('Uncompressed observation exceeds limit')
    with np.load(io.BytesIO(raw), allow_pickle=False) as data:
        packet = {k: data[k] for k in required}
    if packet['rgb'].dtype != np.uint8 or packet['rgb'].shape != (720, 1280, 3):
        raise ValueError('RGB must be uint8 1280x720')
    if packet['depth_m'].shape != (720, 1280) or packet['depth_m'].dtype.kind != 'f':
        raise ValueError('Depth must be float image-plane metres')
    for name, shape in [('base_world', (3,)), ('goal_world_xy', (2,)), ('intrinsic', (3, 3)),
                        ('yaw_rad', ()), ('captured_unix_s', ())]:
        if packet[name].shape != shape or not np.isfinite(packet[name]).all():
            raise ValueError('Invalid ' + name)
    if packet['intrinsic'][0, 0] <= 0 or packet['intrinsic'][1, 1] <= 0:
        raise ValueError('Invalid focal length')
    return packet


def create_app(samples=None, floor_z=None, guidance_scale=.5, commit_horizon=None, depth_memory_frames=0, route_guide=False, route_guard=False):
    if not 0 <= guidance_scale <= 1 or (commit_horizon is not None and not .6 <= commit_horizon <= 2.):
        raise ValueError('Outside simulation planner parameter envelope')
    if not isinstance(depth_memory_frames,int) or not 0 <= depth_memory_frames <= 16:
        raise ValueError('Invalid depth memory size')
    if (route_guide and floor_z is None) or (route_guard and not route_guide):
        raise ValueError('Guide/guard needs an explicit floor and route guide')
    app = Flask(__name__)
    app.config['MAX_CONTENT_LENGTH'] = 12 * 1024 * 1024
    state = {'runtime': None, 'sequence': 0, 'intrinsic': None, 'depth_memory':[]}
    guide=DepthRoute() if route_guide else None
    lock = threading.Lock()
    config = {'samples':samples,'floor_z':floor_z,'guidance_scale':guidance_scale,'commit_horizon_m':commit_horizon,
              'depth_memory_frames':depth_memory_frames,'route_guide':route_guide,'route_guard':route_guard,'reset_seed':42,'source_sha256':{name:hashlib.sha256((ROOT/'jm_xnavdp'/name).read_bytes()).hexdigest()
                   for name in ('server.py','runtime.py','physical_planner.py','depth_route.py')}}

    @app.get('/health')
    def health():
        return jsonify(status='ready', model_loaded=state['runtime'] is not None,
                       robot_control_enabled=False, sequence=state['sequence'], configuration=config)

    @app.post('/reset')
    def reset():
        with lock:
            if state['runtime'] is not None:
                state['runtime'].agent.reset(1)
                import torch
                torch.manual_seed(42)
                np.random.seed(42)
            state['sequence'] = 0
            state['depth_memory'].clear()
            if guide is not None: guide.reset()
        return jsonify(reset=True, robot_control_enabled=False)

    @app.post('/infer')
    def infer():
        try:
            packet = decode_packet(request.get_data())
            with lock:
                if state['runtime'] is None:
                    state['runtime'] = Runtime(packet['intrinsic'])
                    state['intrinsic'] = packet['intrinsic'].copy()
                elif not np.allclose(state['intrinsic'], packet['intrinsic'], rtol=0, atol=1e-6):
                    raise ValueError('Camera calibration changed; restart service')
                started = time.perf_counter()
                base, yaw = packet['base_world'], float(packet['yaw_rad'])
                goal = local_goal(base[:2], yaw, packet['goal_world_xy'])
                route=None
                if guide is not None:
                    c,s=np.cos(yaw),np.sin(yaw)
                    sensor_cloud=depth_points(packet['depth_m'],packet['intrinsic'],base[2]-floor_z)
                    route=guide.update(sensor_cloud@np.array([[c,s],[-s,c]])+base[:2],base[:2],packet['goal_world_xy'])
                    if not route['pass']:
                        state['sequence']+=1
                        return jsonify(simulation_selection={'pass':False,'reason':route['reason']},depth_guide=route,
                            robot_control_enabled=False,service_configuration=config,sequence=state['sequence']-1)
                    goal=local_goal(base[:2],yaw,route['subgoal_world_xy'])
                result = state['runtime'].infer(packet['rgb'], packet['depth_m'], goal, base, yaw,
                    samples=samples, commit_guidance=floor_z is None, guidance_scale=guidance_scale)
                result['depth_guide']=route
                result['final_goal_world_xy']=packet['goal_world_xy'].tolist()
                result['model_goal_local_m']=goal.tolist()
                if floor_z is not None:
                    cloud = depth_points(packet['depth_m'], packet['intrinsic'], base[2]-floor_z)
                    current_count=len(cloud)
                    if depth_memory_frames:
                        c,s=np.cos(yaw),np.sin(yaw)
                        rotation=np.array([[c,s],[-s,c]])
                        state['depth_memory'].append(cloud@rotation+base[:2])
                        state['depth_memory']=state['depth_memory'][-depth_memory_frames:]
                        observed_world=np.concatenate(state['depth_memory'])
                        cloud=(observed_world-base[:2])@rotation.T
                        cloud=cloud[np.linalg.norm(cloud,axis=1)<5.]
                    result['depth_surface_points_current']=current_count
                    result['depth_surface_points_with_memory']=len(cloud)
                    selection = select_path(result['candidates_local_m'], result['q_values'], goal, cloud,
                                            commit_horizon=commit_horizon)
                    result['simulation_selection'] = selection
                    result['commit_horizon_m'] = commit_horizon
                    if route_guard:
                        c,s=np.cos(yaw),np.sin(yaw); rotation=np.array([[c,s],[-s,c]])
                        guide_local=(np.asarray(route['path_world_xy'])-base[:2])@rotation.T
                        ref=prefix_by_length(guide_local,commit_horizon or .8)
                        arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(ref,axis=0),axis=1))]
                        samples_at=np.linspace(0.,arc[-1],25)[1:]
                        proposal=np.c_[np.interp(samples_at,arc,ref[:,0]),np.interp(samples_at,arc,ref[:,1]),np.zeros(24)]
                        checked=select_path(proposal[None],np.array([0.]),goal,cloud)
                        deviation=None
                        if selection['pass']:
                            a=densify(np.asarray(selection['path_body_xy'])); b=densify(guide_local)
                            deviation=float(np.sqrt(((a[:,None]-b[None])**2).sum(-1)).min(axis=1).max())
                        if selection['pass'] and deviation<=.10:
                            execution={**selection,'source':'XNAVDP_MODEL','guide_deviation_m':deviation}
                        else:
                            execution={**checked,'source':'DEPTH_ASTAR_REFERENCE','selected_id':None,
                                       'guide_deviation_m':deviation,'fallback_reason':'model rejected or outside 10cm guide corridor'}
                        result['execution_selection']=execution
                        selection=execution
                    if selection['pass']:
                        selected = (np.asarray(result['candidates_local_m'])[selection['selected_id']]
                                    if selection.get('selected_id') is not None else proposal)
                        if commit_horizon is not None or route_guard:
                            # Guidance follows only the accepted segment, not an unsafe tail.
                            path=np.asarray(selection['path_body_xy'])
                            arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]
                            sample=np.linspace(0.,arc[-1],25)[1:]
                            selected=np.c_[np.interp(sample,arc,path[:,0]),np.interp(sample,arc,path[:,1]),np.zeros(24)]
                        q = np.array([[0., 0., np.sin(yaw/2), np.cos(yaw/2)]])
                        state['runtime'].agent._update_last_trajectory_state(selected[None], base[None], q)
                        result['guidance_committed_candidate'] = selection['selected_id']
                        result['guidance_committed_source'] = selection.get('source','XNAVDP_MODEL')
                    else:
                        # Next episode must explicitly reset; no rejected seed reuse.
                        result['guidance_committed_candidate'] = None
                result['world_path_xy'] = world_path(result['trajectory_local_m'], base[:2], yaw).tolist()
                age = time.time() - float(packet['captured_unix_s'])
                result['command_gate'] = propose_command(result['trajectory_local_m'], age)
                result['observation_age_s'] = age
                result['request_wall_s'] = time.perf_counter() - started
                result['sequence'] = state['sequence']
                state['sequence'] += 1
                result['robot_control_enabled'] = False
                result['model_sha256'] = state['runtime'].audit['model_sha256']
                result['service_configuration'] = config
                return jsonify(result)
        except (ValueError, zipfile.BadZipFile, OSError) as exc:
            return jsonify(error=str(exc), robot_control_enabled=False), 400
        except Exception as exc:
            # Do not return a stale cached trajectory after an inference failure.
            app.logger.exception('X-NavDP inference failed')
            return jsonify(error=str(exc), robot_control_enabled=False), 500

    return app


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8891)
    parser.add_argument('--samples', type=int, choices=[8, 16, 32], default=None)
    parser.add_argument('--floor-z', type=float, default=None,
                        help='Enable hard-gated simulation proposals at this fixed world floor height')
    parser.add_argument('--guidance-scale', type=float, default=.5)
    parser.add_argument('--commit-horizon', type=float, default=None,
                        help='Simulation-only receding path horizon in metres; full raw candidates retained')
    parser.add_argument('--depth-memory-frames', type=int, default=0,
                        help='Retain observed depth surfaces across this many planner frames; conservative static-scene audit')
    parser.add_argument('--route-guide', action='store_true',help='Explicit hybrid: retained RGB-D A* subgoals -> X-NavDP -> gait')
    parser.add_argument('--route-guard', action='store_true',help='Explicit hybrid fallback to checked sensor A* reference when model cuts the route')
    args = parser.parse_args()
    if args.floor_z is not None and args.samples is None:
        parser.error('--floor-z requires --samples')
    create_app(args.samples, args.floor_z, args.guidance_scale, args.commit_horizon, args.depth_memory_frames,args.route_guide,args.route_guard).run(host='127.0.0.1', port=args.port, debug=False, use_reloader=False, threaded=True)
