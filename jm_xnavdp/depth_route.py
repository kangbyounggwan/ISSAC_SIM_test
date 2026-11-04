"""Static-scene RGB-D surface route guide, NOT X-NavDP or a free-space certificate.

Unknown grid cells are tentative, not certified free. Only sensor surfaces
enter this map. Simulation geometry remains a separate abort-only check.
"""
import heapq
import math
import numpy as np
from scipy.ndimage import distance_transform_edt


class DepthRoute:
    def __init__(self, resolution=.10, clearance=.65):
        self.resolution=resolution; self.clearance=clearance
        self.reset()

    def reset(self):
        self.start=None; self.goal=None; self.points={}

    def update(self, cloud_world, position, goal):
        pos=np.asarray(position)[:2]; goal=np.asarray(goal)[:2]
        if self.start is None:
            self.start=pos.copy(); self.goal=goal.copy()
            self.low=np.minimum(pos,goal)-2.5; self.high=np.maximum(pos,goal)+2.5
        if not np.allclose(goal,self.goal,atol=1e-6,rtol=0): raise ValueError('Final goal changed')
        for point in np.asarray(cloud_world):
            if np.all(point>=self.low) and np.all(point<=self.high):
                key=tuple(np.floor(point/.05).astype(int)); self.points[key]=point
        shape=np.ceil((self.high-self.low)/self.resolution).astype(int)+1
        occupied=np.zeros(tuple(shape),dtype=bool)
        if self.points:
            cells=np.rint((np.asarray(list(self.points.values()))-self.low)/self.resolution).astype(int)
            occupied[cells[:,0],cells[:,1]]=True
        distances=distance_transform_edt(~occupied)*self.resolution if occupied.any() else np.full(tuple(shape),np.inf)
        free=distances >= self.clearance+self.resolution*math.sqrt(2)/2
        def cell(p): return tuple(np.rint((p-self.low)/self.resolution).astype(int))
        def valid(c): return 0<=c[0]<shape[0] and 0<=c[1]<shape[1] and free[c]
        source,target=cell(pos),cell(goal)
        # Planning padding is a preference; actual body clearance remains .38m.
        # Permit a short exit connector when tracking error enters that padding.
        if 0<=source[0]<shape[0] and 0<=source[1]<shape[1] and distances[source]>=.38+self.resolution*math.sqrt(2)/2:
            ii,jj=np.indices(tuple(shape))
            near=((ii-source[0])**2+(jj-source[1])**2)*self.resolution**2 <= .5**2
            free |= near & (distances >= .38+self.resolution*math.sqrt(2)/2)
        if not valid(source) or not valid(target):
            return {'pass':False,'reason':'DEPTH_ROUTE_ENDPOINT_BLOCKED','observed_voxels':len(self.points)}
        def world(c): return self.low+np.array(c)*self.resolution
        def line_ok(a,b):
            samples=np.linspace(a,b,max(2,int(np.ceil(np.linalg.norm(b-a)/.035))+1))
            return all(valid(cell(p)) for p in samples)
        if not line_ok(pos,world(source)) or not line_ok(world(target),goal):
            return {'pass':False,'reason':'DEPTH_ROUTE_ENDPOINT_CONNECTION_BLOCKED'}
        queue=[(float(np.linalg.norm(pos-goal)),0.,source)]; cost={source:0.}; parent={}
        directions=[(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]
        while queue:
            _,g,c=heapq.heappop(queue)
            if g>cost[c]+1e-9: continue
            if c==target: break
            for dx,dy in directions:
                n=(c[0]+dx,c[1]+dy)
                if not valid(n): continue
                if dx and dy and (not valid((c[0]+dx,c[1])) or not valid((c[0],c[1]+dy))): continue
                ng=g+self.resolution*math.hypot(dx,dy)
                if ng<cost.get(n,float('inf')):
                    cost[n]=ng; parent[n]=c
                    heapq.heappush(queue,(ng+np.linalg.norm(world(n)-goal),ng,n))
        if target not in cost: return {'pass':False,'reason':'DEPTH_ROUTE_NO_PATH'}
        chain=[target]
        while chain[-1]!=source: chain.append(parent[chain[-1]])
        raw=np.vstack([pos,[world(c) for c in chain[::-1]],goal])
        # Visibility shortcut is checked in the inflated sensor grid, never through obstacles.
        reduced=[raw[0]]; i=0
        while i<len(raw)-1:
            j=len(raw)-1
            while j>i+1 and not line_ok(raw[i],raw[j]): j-=1
            reduced.append(raw[j]); i=j
        path=np.asarray(reduced)
        arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]
        length=min(1.4,arc[-1]); idx=min(np.searchsorted(arc,length,side='right'),len(path)-1)
        subgoal=path[idx-1]+(path[idx]-path[idx-1])*(length-arc[idx-1])/max(arc[idx]-arc[idx-1],1e-8)
        return {'pass':True,'path_world_xy':path.tolist(),'subgoal_world_xy':subgoal.tolist(),
                'observed_voxels':len(self.points),'clearance_m':self.clearance,'resolution_m':self.resolution,
                'final_goal_world_xy':goal.tolist(),'method':'A* on retained RGB-D surfaces; not model-only',
                'start_connector_radius_m':.5,'start_connector_min_clearance_m':.38,
                'unknown_cells':'tentative traversable; NOT verified free','static_scene_only':True}
