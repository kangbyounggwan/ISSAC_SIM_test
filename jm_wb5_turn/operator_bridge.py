"""Optional local desktop telemetry bridge; no network or robot control authority."""
import json
import os
from pathlib import Path
import time


class OperatorBridge:
    def __init__(self,out,enabled=False):
        self.out=Path(out)
        self.enabled=enabled
        self.sequence=0
        self.write_errors=0

    def cancelled(self):
        return self.enabled and (self.out/'cancel.request').is_file()

    def _replace(self,path,write):
        temp=path.with_name(path.name+'.tmp')
        try:
            write(temp)
            os.replace(temp,path)
        except OSError:
            self.write_errors+=1

    def publish(self,state):
        if not self.enabled: return
        self.sequence+=1
        payload=dict(state,sequence=self.sequence,wall_time_unix=time.time(),
                     mode='physical_simulation',write_errors=self.write_errors)
        self._replace(self.out/'live_status.json',
                      lambda p:p.write_text(json.dumps(payload),encoding='utf-8'))

    def frame(self,image,kind='third'):
        if not self.enabled: return
        names={'third':'live_camera.jpg','rgb':'live_rgb.jpg','depth':'live_depth.jpg','lidar':'live_lidar.jpg'}
        if kind not in names: raise ValueError('Unknown sensor view')
        self._replace(self.out/names[kind],lambda p:image.save(p,format='JPEG',quality=87))

    def sensors(self,state):
        if not self.enabled: return
        self._replace(self.out/'live_sensors.json',lambda p:p.write_text(json.dumps(state),encoding='utf-8'))
