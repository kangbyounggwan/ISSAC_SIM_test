#!/usr/bin/env python3
"""Publish only selected experiment artifacts to the existing localhost GUI."""
import argparse
import json
from pathlib import Path
import shutil

p=argparse.ArgumentParser()
p.add_argument('--observations',type=Path,required=True)
p.add_argument('--foundationpose',type=Path,required=True)
p.add_argument('--graspgenx',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
a.output.mkdir(parents=True,exist_ok=True)
capture=json.loads((a.observations/'capture_report.json').read_text())
def read_result(folder):
    path=folder/'report.json'
    return json.loads(path.read_text()) if path.exists() else {'status':'pending','frames':[]}
pose=read_result(a.foundationpose)
grasp=read_result(a.graspgenx)
report={'capture':capture,'foundationpose':pose,'graspgenx':grasp}
(a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
for item in capture['frames']:
    name=item['id']
    dest=a.output/name
    dest.mkdir(exist_ok=True)
    for file in ['rgb.png','mask.png','gt_overlay.png','frame.json']:
        shutil.copy2(a.observations/name/file,dest/file)
    for folder,label in [(a.foundationpose,'foundationpose'),(a.graspgenx,'graspgenx')]:
        src=folder/(name+'_overlay.png')
        if src.exists(): shutil.copy2(src,dest/(label+'.png'))
    src=a.graspgenx/(name+'_grasps.npz')
    if src.exists(): shutil.copy2(src,dest/'grasps.npz')

html='''<!doctype html>
<html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><base href="/task1/">
<title>Task 1 · 첫 모델 연결 실험</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#111822;color:#e8eef5;font:15px system-ui,sans-serif;line-height:1.6}main{max-width:1400px;margin:auto;padding:28px}a{color:#95caff}h1{font-size:28px;margin:12px 0}h2{font-size:18px;margin:0 0 10px}p{color:#b8c7d7}nav{display:flex;gap:10px;flex-wrap:wrap;margin:22px 0}button{font:inherit;color:#dae5f1;background:#223246;border:1px solid #506780;border-radius:7px;padding:8px 18px;cursor:pointer}button[aria-pressed=true]{background:#14628a;border-color:#7acbff}.cards{display:grid;grid-template-columns:repeat(3,1fr);gap:15px}.card,.panel{background:#1b2736;border:1px solid #35475b;border-radius:10px;padding:18px}.number{font-size:26px;font-weight:650;color:#a4d8ff}.grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:18px}.panel img{display:block;width:100%;height:auto;border-radius:5px;background:#121820}.caption{font-size:13px;margin-bottom:0}.note{background:#2d2b20;border-left:3px solid #dcc56f;padding:14px 18px;color:#eadfad}.metrics{margin:18px 0;color:#c1d5e8}footer{margin-top:24px;font-size:13px}table{width:100%;border-collapse:collapse;margin:20px 0}td,th{text-align:left;padding:10px;border-bottom:1px solid #35475b}th{color:#9db4cb} @media(max-width:800px){.cards,.grid{grid-template-columns:1fr}main{padding:16px}table{font-size:12px}}
</style><main>
<a href="/">← 실제 Isaac Sim 화면</a>
<h1>Task 1 · 첫 모델 연결 실험</h1>
<p>공장 장면의 맨 위 PCB 한 장을 세 방향에서 관찰했습니다. 저장된 RGB-D를 두 모델에 각각 입력한 결과입니다.</p>
<div class="cards"><div class="card"><h2>관측 데이터</h2><div class="number">3개 시점</div><span>RGB · 깊이 · 정답 마스크 · 좌표</span></div><div class="card"><h2>FoundationPose</h2><div class="number" id="poseStatus">확인 중</div><span>각 영상에서 물체 자세를 독립 추정</span></div><div class="card"><h2>GraspGen-X</h2><div class="number" id="graspStatus">확인 중</div><span>임시 Franka 그리퍼 · 확산 모델</span></div></div>
<p class="note">초기 연결 검증입니다. 마스크는 시뮬레이터 정답이며, PCB는 60×46×2cm 직육면체입니다. 파지 후보는 H1의 도달성·충돌·접촉 검증을 거치지 않았습니다. 높은 모델 점수가 실제 파지 성공을 뜻하지 않습니다.</p>
<nav aria-label="관측 시점"><button data-frame="frame_000" aria-pressed="true">시점 1</button><button data-frame="frame_001" aria-pressed="false">시점 2</button><button data-frame="frame_002" aria-pressed="false">시점 3 · 가림</button></nav>
<div class="metrics" id="metrics"></div>
<div class="grid"><section class="panel"><h2>원본 RGB</h2><img id="rgb" alt="공장 PCB 관측"><p class="caption">로봇이 이동한 영상이 아닌 고정 관찰 시점입니다.</p></section><section class="panel"><h2>정답 마스크와 물체 상자</h2><img id="gt" alt="정답 마스크와 PCB 경계"><p class="caption">주황색은 보이는 대상 영역, 노란 선은 원본 USD의 3D 경계입니다.</p></section><section class="panel"><h2>FoundationPose 자세 추정</h2><img id="pose" alt="자세 추정 준비 중"><p class="caption"><span style="color:#ff41d2">자홍색 실선: 정답</span> · <span style="color:#00e1ff">청록색 점선: 추정</span>. 회전 오차는 직육면체의 대칭을 고려합니다.</p></section><section class="panel"><h2>GraspGen-X 상위 파지 후보</h2><img id="grasp" alt="파지 후보"><p class="caption">상위 5개의 좌표축 표시입니다. 파란 선은 접근 방향, 노란 선은 닫히는 방향입니다. 실제 그리퍼 형상은 Isaac Sim의 ‘PCB 파지 후보’ 버튼으로 확인하세요.</p></section></div>
<table><thead><tr><th>시점</th><th>정답과 위치 차이</th><th>대칭 고려 회전 차이</th><th>자세 추정 시간</th><th>파지 후보 생성 시간</th></tr></thead><tbody id="rows"></tbody></table>
<p class="caption">시간은 이 서버에서 이번 실행에 측정한 값이며, 모델 로딩과 센서 획득은 제외합니다. 첫 실행에는 초기화 비용이 포함될 수 있습니다. 세 장의 결과를 일반적인 성능 수치로 해석하지 않습니다.</p>
<footer><a href="report.json">전체 결과 JSON</a> · <a id="download" href="frame_000/grasps.npz">현재 시점 파지 행렬·점수 다운로드</a><p>다음 단계: 대상 분할 모델 연결, 실제 PCB 형상 확보, 최종 팔·그리퍼에 대한 IK 및 충돌 검증. 보행 정책과 UniLM-Nav 공식 코드는 아직 연결하지 않았습니다.</p></footer>
</main><script>
fetch('report.json').then(r=>r.json()).then(d=>{
const by=(arr,key,value)=>arr.find(x=>x[key]===value);
const pf=d.foundationpose.frames,gf=d.graspgenx.frames;
document.querySelector('#poseStatus').textContent=d.foundationpose.status==='success'?'추론 완료':'설치·검증 중';
document.querySelector('#graspStatus').textContent=gf.length?`${gf[0].num_candidates}개 / 시점`:'준비 중';
function select(name){document.querySelectorAll('[data-frame]').forEach(b=>b.setAttribute('aria-pressed',String(b.dataset.frame===name)));
for(const [id,file] of [['rgb','rgb.png'],['gt','gt_overlay.png'],['pose','foundationpose.png'],['grasp','graspgenx.png']]){const im=document.querySelector('#'+id);im.hidden=id==='pose'&&!by(pf,'frame',name);if(!im.hidden)im.src=name+'/'+file;}
const c=by(d.capture.frames,'id',name),g=by(gf,'frame',name);
document.querySelector('#metrics').textContent=`대상 깊이 점 ${c.valid_target_pixels.toLocaleString()}개 · 원본 표면과 깊이 차이(99백분위) ${(c.depth_mesh_residual_p99_m*1000).toFixed(2)}mm`+(g?` · 후보 최고 점수 ${g.score_max.toFixed(3)} (성공 확률 아님)`:'');
document.querySelector('#download').href=name+'/grasps.npz';}
document.querySelectorAll('[data-frame]').forEach(b=>b.onclick=()=>select(b.dataset.frame));
for(const c of d.capture.frames){const p=by(pf,'frame',c.id),g=by(gf,'frame',c.id);const tr=document.createElement('tr');for(const value of [c.id,p?(p.translation_error_m*1000).toFixed(1)+' mm':'준비 중',p?p.box_symmetry_rotation_error_deg.toFixed(1)+'°':'—',p?p.inference_seconds.toFixed(2)+' s':'—',g?g.inference_seconds.toFixed(2)+' s':'—']){const td=document.createElement('td');td.textContent=value;tr.append(td)}document.querySelector('#rows').append(tr)}select('frame_000');
}).catch(e=>{document.querySelector('#metrics').textContent='결과를 읽지 못했습니다: '+e.message});
</script></html>'''
(a.output/'index.html').write_text(html)
print(a.output/'index.html')
