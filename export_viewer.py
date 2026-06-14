#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""export_viewer.py — world_geometry 를 standalone three.js HTML 뷰어로 내보내기.
층 선택 + 건축/구조 토글 UI 포함. viewer.html 더블클릭 → 브라우저에서 3D 확인.
(three.js CDN 로드 → 최초 1회 인터넷 필요)
"""
import json
from world_geometry import build_scene, SITE, FLOORS


def hx(c):
    return (int(c[0] * 255) << 16) | (int(c[1] * 255) << 8) | int(c[2] * 255)


def export(html_path="viewer.html"):
    parts = build_scene()
    items = []
    for p in parts:
        d = dict(fl=p["floor"], ct=p["cat"], k=hx(p["color"]))
        if p["type"] == "box":
            d["t"] = "b"; d["c"] = [round(v, 3) for v in p["center"]]
            d["s"] = [round(v, 3) for v in p["size"]]
            if p.get("rot"):
                d["q"] = [round(v, 5) for v in p["rot"]]
        else:
            d["t"] = "m"; d["p"] = [[round(v, 3) for v in pt] for pt in p["points"]]; d["f"] = p["faces"]
        items.append(d)
    data = json.dumps(dict(it=items, fl=FLOORS, cx=SITE["w"] / 2, cy=SITE["d"] / 2),
                      separators=(",", ":"))

    html = r"""<!DOCTYPE html>
<html lang="ko"><head><meta charset="utf-8"><title>JM Electronics 시화공장 — Isaac Sim World</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
 html,body{margin:0;height:100%;overflow:hidden;background:#aab7c4;font-family:system-ui,'Malgun Gothic',sans-serif}
 #ui{position:absolute;top:10px;left:12px;background:rgba(255,255,255,.9);padding:10px 12px;border-radius:10px;
   font-size:13px;color:#0b1f33;max-width:240px;box-shadow:0 1px 6px rgba(0,0,0,.15)}
 #ui b{font-size:14px} .btn{display:block;width:100%;text-align:left;margin:3px 0;padding:5px 8px;border:1px solid #bcc6d2;
   background:#fff;border-radius:6px;font-size:12px;cursor:pointer;color:#16324f}
 .btn.on{background:#1f4e85;color:#fff;border-color:#1f4e85}
 .row{display:flex;gap:6px;margin:6px 0}
 .row .btn{margin:0}
 #hint{position:absolute;bottom:10px;left:12px;background:rgba(255,255,255,.85);padding:6px 10px;border-radius:8px;
   font-size:12px;color:#0b1f33}
</style></head><body>
<div id="ui">
 <b>제이엠일렉트로닉스 시화공장</b>
 <div style="font-size:11px;color:#456;margin:2px 0 8px">CAD 기반 Isaac Sim 월드 · 층 선택</div>
 <div class="row"><button class="btn on" data-cat="all">전체</button>
   <button class="btn" data-cat="arch">건축</button><button class="btn" data-cat="struct">구조</button>
   <button class="btn" data-cat="equip">장비</button></div>
 <div id="floors"></div>
</div>
<div id="hint">드래그 회전 · 휠 확대 · 우클릭 이동 · <span style="color:#c00">→</span>=북(N)</div>
<script type="importmap">{"imports":{
 "three":"https://unpkg.com/three@0.160.0/build/three.module.js",
 "three/addons/":"https://unpkg.com/three@0.160.0/examples/jsm/"}}</script>
<script type="module">
import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
const D = __DATA__;
const scene=new THREE.Scene(); scene.background=new THREE.Color(0xaab7c4);
scene.fog=new THREE.Fog(0xaab7c4,200,520);
const cam=new THREE.PerspectiveCamera(52,innerWidth/innerHeight,0.1,3000); cam.up.set(0,0,1);
cam.position.set(D.cx-62,D.cy-98,72);
const rn=new THREE.WebGLRenderer({antialias:true}); rn.setSize(innerWidth,innerHeight);
rn.setPixelRatio(Math.min(devicePixelRatio,2)); document.body.appendChild(rn.domElement);
const ctr=new OrbitControls(cam,rn.domElement); ctr.target.set(D.cx,D.cy,6); ctr.enableDamping=true; ctr.update();
scene.add(new THREE.HemisphereLight(0xdfe7f2,0x55606e,0.95));
const sun=new THREE.DirectionalLight(0xfff4e0,1.4); sun.position.set(D.cx+55,D.cy-45,140); scene.add(sun);
const mats={}; function mat(k){ if(!mats[k]) mats[k]=new THREE.MeshLambertMaterial({color:k}); return mats[k]; }
const objs=[];
for(const o of D.it){
 let m;
 if(o.t==='b'){
   const g=new THREE.BoxGeometry(o.s[0],o.s[1],o.s[2]);
   m=new THREE.Mesh(g,mat(o.k)); m.position.set(o.c[0],o.c[1],o.c[2]);
   if(o.q) m.quaternion.set(o.q[0],o.q[1],o.q[2],o.q[3]);
 } else {
   const pos=[]; o.f.forEach(f=>{for(let i=1;i<f.length-1;i++){[f[0],f[i],f[i+1]].forEach(ix=>{const p=o.p[ix];pos.push(p[0],p[1],p[2]);});}});
   const g=new THREE.BufferGeometry(); g.setAttribute('position',new THREE.Float32BufferAttribute(pos,3)); g.computeVertexNormals();
   m=new THREE.Mesh(g,new THREE.MeshLambertMaterial({color:o.k,side:THREE.DoubleSide}));
 }
 m.userData={fl:o.fl,ct:o.ct}; scene.add(m); objs.push(m);
}
scene.add(new THREE.ArrowHelper(new THREE.Vector3(0,1,0),new THREE.Vector3(2,2,0.3),14,0xcc0000,5,2.8));
let curFloor='all', curCat='all';
function apply(){
 for(const m of objs){
   const f=m.userData.fl, c=m.userData.ct;
   const showFloor = (curFloor==='all') || (f===curFloor) || (f==='site');
   const showCat = (curCat==='all') || (c===curCat) || (c==='site');
   m.visible = showFloor && showCat;
 }
}
const fc=document.getElementById('floors');
const allBtn=document.createElement('button'); allBtn.className='btn on'; allBtn.textContent='전체 (모든 층)'; allBtn.dataset.fl='all'; fc.appendChild(allBtn);
for(const [code,label] of D.fl){ const b=document.createElement('button'); b.className='btn'; b.textContent=label; b.dataset.fl=code; fc.appendChild(b); }
fc.addEventListener('click',e=>{ if(!e.target.dataset.fl)return;
 fc.querySelectorAll('.btn').forEach(b=>b.classList.remove('on')); e.target.classList.add('on');
 curFloor=e.target.dataset.fl; apply(); });
document.querySelectorAll('[data-cat]').forEach(b=>b.addEventListener('click',e=>{
 document.querySelectorAll('[data-cat]').forEach(x=>x.classList.remove('on')); e.target.classList.add('on');
 curCat=e.target.dataset.cat; apply(); }));
addEventListener('resize',()=>{cam.aspect=innerWidth/innerHeight;cam.updateProjectionMatrix();rn.setSize(innerWidth,innerHeight);});
(function loop(){requestAnimationFrame(loop);ctr.update();rn.render(scene,cam);})();
</script></body></html>"""
    html = html.replace("__DATA__", data)
    with open(html_path, "w", encoding="utf-8") as f:
        f.write(html)
    print("HTML:", html_path, " items=%d  (%.0f KB)" % (len(items), len(data) / 1024))


if __name__ == "__main__":
    export(r"C:\issac sim test\viewer.html")
