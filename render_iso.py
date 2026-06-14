#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""render_iso.py — world_geometry 를 PIL 만으로 3D 음영(축측투영) 렌더 (GPU/USD 불필요)."""
import math
from PIL import Image, ImageDraw
from world_geometry import build_scene, box_corners, SITE

# ----- 카메라(직교 축측투영) : 남서-상공에서 북동-하향 응시 -----
def normalize(v):
    n = math.sqrt(sum(c * c for c in v)) or 1.0
    return tuple(c / n for c in v)

def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])

def dot(a, b):
    return sum(x*y for x, y in zip(a, b))

D = normalize((1.0, 1.2, -0.85))      # 시선 방향(카메라→씬)
U = normalize(cross(D, (0, 0, 1)))    # 화면 오른쪽
V = normalize(cross(U, D))            # 화면 위
LIGHT = normalize((-0.45, -0.35, -0.82))  # 햇빛 방향

BOX_FACES = [(0,1,2,3),(4,7,6,5),(0,4,5,1),(1,5,6,2),(2,6,7,3),(3,7,4,0)]

def face_normal(pts):
    nx=ny=nz=0.0
    n=len(pts)
    for i in range(n):
        x0,y0,z0=pts[i]; x1,y1,z1=pts[(i+1)%n]
        nx+=(y0-y1)*(z0+z1); ny+=(z0-z1)*(x0+x1); nz+=(x0-x1)*(y0+y1)
    return normalize((nx,ny,nz))

def render(path, W=1500, H=1000):
    parts = build_scene()
    faces = []   # (depth, screen_pts, fill)
    for p in parts:
        col = p["color"]
        if p["type"] == "box":
            verts=box_corners(p)
            polys=[[verts[i] for i in f] for f in BOX_FACES]
        else:
            polys=[[p["points"][i] for i in f] for f in p["faces"]]
        for poly in polys:
            nrm=face_normal(poly)
            if dot(nrm, D) > -0.02:    # 카메라 반대 향한 면 제거(후면 컬링)
                continue
            shade=0.32 + 0.68*max(0.0, dot(nrm, [-l for l in LIGHT]))
            fill=tuple(min(255,int(c*255*shade)) for c in col)
            spts=[(dot(v,U), dot(v,V)) for v in poly]
            depth=sum(dot(v,D) for v in poly)/len(poly)
            faces.append((depth, spts, fill))
    # 후→전 정렬(painter's): depth 큰(=먼) 것 먼저
    faces.sort(key=lambda f:-f[0])
    # 화면 스케일/오프셋
    allx=[x for _,sp,_ in faces for x,_ in sp]; ally=[y for _,sp,_ in faces for _,y in sp]
    minx,maxx=min(allx),max(allx); miny,maxy=min(ally),max(ally)
    pad=40
    sc=min((W-2*pad)/(maxx-minx),(H-2*pad)/(maxy-miny))
    def px(s):
        x,y=s
        return (pad+(x-minx)*sc, H-pad-(y-miny)*sc)
    img=Image.new("RGB",(W,H),(170,183,196))
    dr=ImageDraw.Draw(img,"RGBA")
    for _,sp,fill in faces:
        dr.polygon([px(s) for s in sp], fill=fill, outline=(35,38,42))
    dr.text((20,18),"JM Electronics 시화공장 — Isaac Sim world (CAD 기반)  | view: SW-above, +Y=North",
            fill=(15,25,40))
    img.save(path)
    print("saved", path, img.size, " faces:", len(faces))

if __name__ == "__main__":
    render(r"C:\issac sim test\render_iso.png")
