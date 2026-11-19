"""Convert a real exported JSONL stream without resampling or inventing missing channels.
Also generate a standalone Canvas replay viewer. No serial/BLE or hardware writes."""
import argparse, csv, hashlib, json, math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def convert(source, destination):
    groups={}
    for line in source.read_text(encoding="utf-8-sig").splitlines():
        if not line.strip():continue
        item=json.loads(line)
        groups.setdefault((item.get("webSession"),item.get("streamSequence")),[]).append(item)
    # Longest uninterrupted original stream, never join independent BLE sessions.
    key,rows=max(groups.items(),key=lambda item:len(item[1]))
    destination.parent.mkdir(parents=True,exist_ok=True)
    with destination.open("w",encoding="utf-8",newline="") as f:
        w=csv.writer(f)
        w.writerow(["timestamp_us","left_deg","right_deg","left_deg_s","right_deg_s","left_valid","right_valid"])
        for r in rows:
            values=[r.get(k) for k in ("leftPositionDeg","rightPositionDeg","leftVelocityDegS","rightVelocityDegS")]
            valid=[all(isinstance(values[j],(float,int)) and math.isfinite(values[j]) for j in (i,i+2)) for i in range(2)]
            w.writerow([int(r["timestampMs"])*1000,*[x if isinstance(x,(int,float)) and math.isfinite(x) else 0 for x in values],int(valid[0]),int(valid[1])])
    evidence={"source":str(source.relative_to(ROOT)) if source.is_relative_to(ROOT) else str(source),
        "source_sha256":hashlib.sha256(source.read_bytes()).hexdigest(),"selected_web_session":key[0],
        "selected_stream":key[1],"rows":len(rows),"span_s":(rows[-1]["timestampMs"]-rows[0]["timestampMs"])/1000,
        "resampled":False,"missing_values":"numeric placeholder zero with valid=false; never treated as valid zero",
        "hardware_context":"Historical IDLE/standing export; not a labelled walking/contact dataset",
        "normalized_sha256":hashlib.sha256(destination.read_bytes()).hexdigest()}
    destination.with_suffix(".provenance.json").write_text(json.dumps(evidence,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    print(json.dumps(evidence,ensure_ascii=False))
def viewer(source,destination):
    with source.open(encoding="utf-8") as f: rows=list(csv.DictReader(f))
    if not rows: raise ValueError("No samples to visualize")
    # 同一绘图器接收主机回放和只读UART采集，保留未知值为断线。
    aliases={"q_deg":"position_deg","v_deg_s":"velocity_deg_s","hz":"frequency_hz",
             "limited_preview_nm":"limited_nm"}
    rows=[{aliases.get(k,k):v for k,v in r.items()} for r in rows]
    # Decimate only display points, raw replay CSV stays complete.
    points=[{k:(float(v) if math.isfinite(float(v)) else None) for k,v in r.items()}
            for i,r in enumerate(rows) if (i//2)%2==0]
    data=json.dumps(points,separators=(",",":"),allow_nan=False)
    page=TEMPLATE.replace("__DATA__",data).replace("__SOURCE__",source.name)
    destination.write_text(page,encoding="utf-8")
    print(f"viewer={destination} displayed_rows={len(points)} raw_rows={len(rows)}")
TEMPLATE="""<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>ESP-EXO · AFO离线回放</title>
<style>body{margin:0;background:#091b29;color:#d8eaf5;font:15px system-ui;padding:24px}h1{font-size:24px}p{color:#91adbf}
.controls{position:sticky;top:0;background:#10293bea;padding:16px;border:1px solid #28475c;border-radius:12px;display:flex;gap:15px;align-items:center;flex-wrap:wrap}input[type=range]{flex:1;min-width:160px}
button,select{background:#1b3e53;color:white;border:1px solid #42738c;padding:8px;border-radius:6px}.grid{display:grid;grid-template-columns:repeat(2,minmax(240px,1fr));gap:16px;margin-top:18px}
article{background:#102434;border:1px solid #29485b;border-radius:12px;padding:16px}h2{font-size:16px;margin:0 0 10px}canvas{width:100%;height:210px}small{color:#87aabe}
@media(max-width:720px){.grid{grid-template-columns:1fr}}.tag{color:#ffb454}</style>
<h1>ESP-EXO · AFO / 双高斯回放</h1><p>来源：__SOURCE__ · 主机C算法结果 · <span class="tag">离线仿真，不连接电机</span></p>
<p>蓝色=LEFT，橙色=RIGHT。limited为安全层限制后的建议；此页面不表示力矩已施加。步态零点是同侧屈髋峰，不是足跟着地。</p>
<div class="controls"><button id="play">播放</button><label>窗口 <select id="window"><option>5</option><option selected>10</option><option>20</option><option>60</option></select> 秒</label><input id="time" type="range" min="0" max="60" step=".01"><output id="readout"></output></div>
<div class="grid" id="plots"></div><p id="cursor">拖动时间轴查看历史；曲线由完整CSV抽取显示，原始CSV保留全部数据。</p>
<script>
const rows=__DATA__, keys=[["position_deg","人体髋角 / deg"],["velocity_deg_s","人体髋角速度 / deg/s"],["frequency_hz","AFO频率 / Hz"],["corrected_phase","校正相位 / 周期"],["desired_nm","双高斯目标 / N·m"],["limited_nm","受限建议 / N·m"],["confidence","相位置信度 / 0–1"],["faults","门禁位 / bitmask"]];
const origin=rows[0].timestamp_us/1e6, duration=rows.at(-1).timestamp_us/1e6-origin;
const time=document.querySelector("#time"), win=document.querySelector("#window");time.max=duration;time.value=Math.min(15,duration);
const panels=keys.map(([key,label])=>{const a=document.createElement("article");a.innerHTML="<h2>"+label+"</h2><canvas></canvas><small></small>";document.querySelector("#plots").append(a);return {key,canvas:a.querySelector("canvas"),label:a.querySelector("small")};});
function draw(){const end=+time.value,start=Math.max(0,end-(+win.value));document.querySelector("#readout").value=end.toFixed(2)+" / "+duration.toFixed(2)+" s";
const segment=rows.filter(r=>r.timestamp_us/1e6-origin>=start&&r.timestamp_us/1e6-origin<=end);
for(const p of panels){const c=p.canvas,dpr=devicePixelRatio||1;c.width=c.clientWidth*dpr;c.height=210*dpr;const x=c.getContext("2d");x.scale(dpr,dpr);const w=c.clientWidth,h=210;
let vals=segment.map(r=>r[p.key]).filter(Number.isFinite),lo=Math.min(0,...vals),hi=Math.max(.01,...vals);if(p.key.includes("phase")||p.key==="confidence"){lo=0;hi=1;}const pad=(hi-lo)*.08;lo-=pad;hi+=pad;
x.strokeStyle="#28495e";x.fillStyle="#85a9be";x.font="11px sans-serif";
for(let i=0;i<=4;i++){let y=15+i*42;x.beginPath();x.moveTo(42,y);x.lineTo(w-8,y);x.stroke();x.fillText((hi-(hi-lo)*i/4).toFixed(2),1,y+3);}
for(const side of [0,1]){x.strokeStyle=side?"#ffae50":"#42cfff";x.lineWidth=1.6;x.beginPath();let last=null;
for(const r of segment.filter(r=>r.side===side)){if(!Number.isFinite(r[p.key])){last=null;continue;}const px=42+(r.timestamp_us/1e6-origin-start)/Math.max(.01,end-start)*(w-50),py=15+(hi-r[p.key])/(hi-lo)*168;
if(!last||r.timestamp_us-last.timestamp_us>200000)x.moveTo(px,py);else x.lineTo(px,py);last=r;}x.stroke();}
p.label.textContent=start.toFixed(2)+" s → "+end.toFixed(2)+" s";}}
time.oninput=draw;win.onchange=draw;window.onresize=draw;
let playing=false;document.querySelector("#play").onclick=()=>{playing=!playing;document.querySelector("#play").textContent=playing?"暂停":"播放";};
setInterval(()=>{if(playing){time.value=(+time.value+.05)>duration?0:+time.value+.05;draw();}},50);draw();
</script></html>"""
if __name__=="__main__":
    p=argparse.ArgumentParser();sub=p.add_subparsers(dest="action",required=True)
    for name in ["convert","viewer"]:
        a=sub.add_parser(name);a.add_argument("source",type=Path);a.add_argument("destination",type=Path)
    a=p.parse_args()
    (convert if a.action=="convert" else viewer)(a.source,a.destination)
