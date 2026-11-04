"""Read-only PhysX contact-pair evidence, independent of the filtered force matrix."""
import json
from pathlib import Path
import numpy as np


class ContactAudit:
    def __init__(self,stage,robot_path,out):
        from pxr import PhysxSchema, UsdPhysics, PhysicsSchemaTools
        from omni.physx import get_physx_simulation_interface
        self.robot=robot_path; self.out=Path(out); self.stage=stage
        self.decode=PhysicsSchemaTools.intToSdfPath
        self.interface=get_physx_simulation_interface()
        self.pairs={}; self.frames=0; self.details=[]; self.max_ground_force=0.
        self.inventory=[]
        for prim in stage.Traverse():
            path=str(prim.GetPath())
            if path.startswith(robot_path+'/') and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.)
            if prim.HasAPI(UsdPhysics.CollisionAPI) and (path.startswith(robot_path+'/') or 'RACK_cont_008' in path or '/JMRingWorkcells/H1_1/' in path):
                self.inventory.append(dict(path=path,enabled=UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get(),type=prim.GetTypeName()))
        (self.out/'contact_collider_inventory.json').write_text(json.dumps(self.inventory,indent=2),encoding='utf-8')

    def sample(self,t):
        headers,data=self.interface.get_contact_report()
        self.frames+=1
        for h in headers:
            a,b=str(self.decode(h.collider0)),str(self.decode(h.collider1))
            if not (a.startswith(self.robot+'/') or b.startswith(self.robot+'/')): continue
            key=' | '.join(sorted([a,b]))
            p=self.pairs.setdefault(key,dict(colliders=[a,b],samples=0,points=0,first_s=t,last_s=t,max_impulse_Ns=0.,minimum_separation_m=None))
            p['samples']+=1; p['last_s']=t; p['points']+=int(h.num_contact_data)
            for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data):
                d=data[i]
                impulse=float(np.linalg.norm(np.asarray(d.impulse,dtype=float)))
                sep=float(d.separation)
                p['max_impulse_Ns']=max(p['max_impulse_Ns'],impulse)
                p['minimum_separation_m']=sep if p['minimum_separation_m'] is None else min(p['minimum_separation_m'],sep)
                other=b if a.startswith(self.robot+'/') else a
                if 'RACK' in other or 'JMRingWorkcells' in other:
                    self.details.append(dict(time_s=t,colliders=[a,b],position=list(d.position),normal=list(d.normal),impulse_Ns=list(d.impulse),separation_m=sep))

    def save(self):
        result=dict(sampled_physics_frames=self.frames,pairs=list(self.pairs.values()),rack_contact_points=self.details)
        (self.out/'raw_contacts.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
        return result
