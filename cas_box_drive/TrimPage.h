#pragma once
// TrimPage - the dedicated phone page of cas_box_trimcal (served at "/" from flash).
// Network load: one small status request every 0.5 s; while a run is active also a heartbeat every
// 0.25 s. No LiDAR drawing, no big sensor JSON (the standard page polled ~700 B every 0.25 s).
static const char kTrimPage[] PROGMEM = R"rawpage(<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>Drive + camera</title><style>
*{box-sizing:border-box}body{margin:0;font-family:system-ui,sans-serif;background:#0f1419;color:#e6e6e6;padding-bottom:110px}
.wrap{max-width:560px;margin:0 auto;padding:10px}
h1{font-size:20px;margin:6px 0 8px}.chips span{display:inline-block;padding:3px 9px;border-radius:12px;margin:2px;font-size:13px;background:#2a2f36}
.ok{background:#1f5f3a!important}.bad{background:#7a2525!important}
.card{background:#1a2027;border-radius:12px;padding:12px;margin:10px 0}
.big{font-size:24px;font-weight:700}.mid{font-size:17px}.small{font-size:13px;color:#9aa4ad}
.meter{position:relative;height:26px;background:#2a2f36;border-radius:13px;margin:10px 0 4px;overflow:hidden}
.meter .z{position:absolute;left:50%;top:0;bottom:0;width:2px;background:#e6e6e6}
.meter .b{position:absolute;top:3px;bottom:3px;background:#4ea1ff;border-radius:10px}
.lr{display:flex;justify-content:space-between;font-size:12px;color:#9aa4ad}
.seg{display:flex;gap:6px}.seg button{flex:1}
button{font-size:17px;padding:13px;border-radius:10px;border:0;background:#2f6fd0;color:#fff;width:100%;margin:5px 0}
button.sel{background:#2f6fd0}.seg button:not(.sel){background:#2a2f36}
button:disabled{background:#3a3f46;color:#888}
.step{border-left:4px solid #3a3f46;padding:6px 10px;margin:10px 0}.step.done{border-color:#2fa060}.step.now{border-color:#4ea1ff}
.step b{font-size:16px}
#stop{position:fixed;left:10px;right:10px;bottom:10px;max-width:540px;margin:0 auto;background:#c62828;font-size:24px;padding:20px;font-weight:700}
.res{font-size:20px;font-weight:700;margin:6px 0}.pass{color:#4cd07d}.fail{color:#ff7272}
table{width:100%;border-collapse:collapse;font-size:13px}td,th{padding:4px;border-bottom:1px solid #2a2f36;text-align:left}
details{background:#1a2027;border-radius:12px;padding:10px 12px;margin:10px 0}summary{font-size:17px;font-weight:600}
.pad{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;margin-top:10px}.pad button{padding:22px 0;font-size:24px;background:#3a4656;touch-action:none;user-select:none}
pre{white-space:pre-wrap;font-size:12px;color:#c9d1d9;margin:6px 0}
input{font-size:17px;padding:8px;width:110px;border-radius:8px;border:0}
</style></head><body><div class="wrap">
<h1>Drive + camera &middot; Kit 04</h1>
<div class="chips"><span id="cR">Rover</span><span id="cI">IMU</span><span id="cL">LiDAR</span><span id="cB">Battery</span><span id="cN">Page</span></div>

<div class="card"><div class="big" id="ph">-</div><div class="mid" id="drTxt">-</div>
<div class="meter"><div class="z"></div><div class="b" id="bar"></div></div>
<div class="lr"><span>&larr; turning LEFT</span><span>straight</span><span>turning RIGHT &rarr;</span></div>
<div class="mid" id="fr" style="margin-top:8px">-</div><div class="small" id="msg"></div></div>

<div class="card"><b>Camera link</b> <span class="small">(display only - nothing steers on the camera yet)</span>
<div class="chips" style="margin-top:6px"><span id="kP">Camera</span><span id="kF">Data</span><span id="kS">Strip</span></div>
<div class="mid" id="kTxt" style="margin-top:6px">-</div>
<div class="meter"><div class="z"></div><div class="b" id="kBar"></div></div>
<div class="lr"><span>&larr; strip LEFT</span><span>centre</span><span>strip RIGHT &rarr;</span></div>
<div class="small" id="kNum"></div>
<div class="seg" style="margin-top:6px"><button id="kM0" class="sel">Wide strip (tactile)</button><button id="kM1">Narrow (belt)</button></div></div>

<div class="card"><div class="small">Speed to calibrate (the route uses 0.25)</div>
<div class="seg"><button id="s0">0.20</button><button id="s1" class="sel">0.25</button></div>
<div class="step" id="st1"><b>Step 1 &middot; Rough (AUTO)</b><div class="small" id="t1"></div><button id="b1">Run step 1</button></div>
<div class="step" id="st2"><b>Step 2 &middot; Measure (3 runs)</b><div class="small" id="t2"></div><button id="b2">Run step 2</button></div>
<div class="step" id="st3"><b>Step 3 &middot; Verify</b><div class="small" id="t3"></div><button id="b3">Run step 3</button><div class="res" id="r3"></div></div>
<div class="res" id="fin"></div>
<button id="bSave">Save trim in the box</button><button id="bDl">Download results (CSV)</button></div>

<div class="card"><b>Runs</b><table id="tab"></table></div>

<details><summary>Move rover (hold a button)</summary><div class="small">Only while no run is active. Release = gentle stop. Turns use full power (the rover does not turn below that).</div>
<div class="pad"><span></span><button data-l="0.2" data-r="0.2">&#9650;</button><span></span>
<button data-l="-0.3" data-r="0.3">&#10226;</button><button data-l="-0.15" data-r="-0.15">&#9660;</button><button data-l="0.3" data-r="-0.3">&#10227;</button></div></details>

<details><summary>Debug</summary><pre id="dbg">-</pre>
<button id="bRate">Read rover's stored ratio (T:139, read-only)</button>
<div class="small">Custom run (any trim, measured but not part of the steps)</div>
<div style="display:flex;gap:8px;align-items:center"><input id="cT" type="number" step="0.005" value="0"><button id="bC" style="flex:1">Run custom</button></div>
<button id="bLast">Download last run (10 Hz samples)</button><button id="bClr" style="background:#7a2525">Clear all results</button></details>
</div>
<button id="stop">&#9632; STOP</button>
<script>
const $=id=>document.getElementById(id);let S=null,si=1,run=false,lastOk=0;const SP=['0.20','0.25'];
const PID=Math.floor(Math.random()*1e9);let seq=0,holdT=null;
function chip(id,ok,txt){const e=$(id);e.textContent=txt;e.className=ok?'ok':'bad';}
function go(k,extra){fetch('/trim/start?kind='+k+'&spd='+SP[si]+(extra||''),{cache:'no-store'}).then(()=>{run=true;poll();});}
async function dl(url,name){try{const b=await (await fetch(url,{cache:'no-store'})).blob();const a=document.createElement('a');
 a.href=URL.createObjectURL(b);a.download=name;document.body.appendChild(a);a.click();a.remove();}catch(e){alert('download failed');}}
function f4(x){return x===null||x===undefined?'-':(x>=0?'+':'')+x.toFixed(4);}
function render(){if(!S)return;const p=S.sp[si];run=S.ph>0;
 chip('cR',S.rover,'Rover '+(S.rover?'OK':'--'));chip('cI',S.imu,'IMU '+(S.imu?'OK':'--'));chip('cL',S.lidar,'LiDAR '+(S.lidar?'OK':'--'));
 chip('cB',S.batt>11.3,'Battery '+(S.batt>0?S.batt.toFixed(1)+' V':'?'));
 $('ph').textContent=run?('RUN '+S.kind+' @ '+S.spd.toFixed(2)+' - '+S.phase+' ('+S.t+' s)'):'Ready';
 $('ph').style.color=run?(S.ph==4?'#4cd07d':'#4ea1ff'):'';
 const r=S.rate,a=Math.abs(r);$('drTxt').textContent=a<0.1?'Straight (drift '+r.toFixed(2)+' deg/s)':('Turning '+(r>0?'LEFT ':'RIGHT ')+a.toFixed(2)+' deg/s'+(run?'':' (standing: should be ~0)'));
 const w=Math.min(50,a*10);const bar=$('bar');bar.style.width=w+'%';bar.style.left=(r>0?50-w:50)+'%';bar.style.background=a<S.pass?'#4cd07d':(a<1?'#e0b030':'#ff7272');
 const fOk=S.front>=S.need;$('fr').textContent='Free space ahead (LiDAR): '+(S.front>=9.9?'> 9 m':S.front.toFixed(2)+' m')+(run?'':(fOk?'  - OK':(S.front>=S.needMin?'  - short (run will be shorter)':'  - TOO SHORT, need '+S.need+' m')));
 $('fr').style.color=run?'':(fOk?'#4cd07d':(S.front>=S.needMin?'#e0b030':'#ff7272'));
 $('msg').textContent=(S.why&&!run?('Last run ended: '+S.why+'. '):'')+(S.msg||'');
 const can=!run&&S.rover&&S.imu&&S.lidar&&S.front>=S.needMin;
 $('t1').textContent='Done: '+p.auto+' (do 2-3). Starts at trim '+f4(p.autoTrim)+', adjusts itself while driving.';
 $('t2').textContent='Done: '+p.fixedK+' (do 3). Next run uses trim '+f4(p.nextFixed)+' (fixed).';
 $('b1').textContent='Run step 1 (AUTO)';$('b2').textContent='Run step 2 (trim '+f4(p.nextFixed)+')';
 const sug=p.sug!==null&&p.fit;$('b3').textContent=sug?('Run step 3 (trim '+f4(p.sug)+')'):'Run step 3 (after step 2)';
 $('t3').textContent='Drives with the suggested trim. PASS = drift below '+S.pass+' deg/s.';
 $('b1').disabled=!can;$('b2').disabled=!can||p.auto<1;$('b3').disabled=!can||!sug;
 $('st1').className='step '+(p.auto>=2?'done':'now');$('st2').className='step '+(p.fixedK>=3?'done':(p.auto>=1?'now':''));
 $('st3').className='step '+(p.vN>0&&p.vRate!==null&&Math.abs(p.vRate)<S.pass?'done':(sug?'now':''));
 if(p.vN>0&&p.vRate!==null){const ok=Math.abs(p.vRate)<S.pass;$('r3').textContent=(ok?'PASS':'NOT YET')+': drift '+p.vRate.toFixed(2)+' deg/s'+(ok?'':' - do step 2 once more');$('r3').className='res '+(ok?'pass':'fail');}else $('r3').textContent='';
 $('fin').textContent=p.sug!==null?('Trim for '+SP[si]+': '+f4(p.sug)+(p.fit?' (fitted from '+p.n+' runs)':' (rough estimate)')+(p.saved!==null?' | saved: '+f4(p.saved):'')):'No result yet for '+SP[si];
 $('bSave').disabled=run;$('bDl').disabled=run;$('bC').disabled=!can;$('bLast').disabled=run;$('bClr').disabled=run;$('bRate').disabled=run;
 let t='<tr><th>#</th><th>speed</th><th>kind</th><th>trim</th><th>drift</th><th>s</th><th>end</th></tr>';
 S.runs.slice().reverse().forEach(x=>{t+='<tr><td>'+x[0]+'</td><td>'+x[1].toFixed(2)+'</td><td>'+x[2]+'</td><td>'+f4(x[3])+'</td><td>'+(x[4]===null?'-':x[4].toFixed(2))+'</td><td>'+x[5].toFixed(1)+'</td><td>'+x[6]+'</td></tr>';});
 $('tab').innerHTML=t;const d=S.dbg;
 $('dbg').textContent='trim now '+S.trim.toFixed(4)+'  gyro bias '+d.bias+' deg/s\nroll '+d.roll+'  pitch '+d.pitch+'  yaw '+d.yaw+'\nahead: LiDAR '+d.lidarF+' m, front ultrasonic '+d.usF+' cm (only counts below 25 cm - it hears the floor at ~60-70 cm)\nultrasonic F/L/R/B/FA: '+d.us.join(' / ')+' cm\nrover rtt '+d.rtt+' ms, ok '+d.ok+', fail '+d.fail+'\nheap '+d.heap+' kB (min '+d.minheap+' kB), uptime '+d.up+' s\nlast reset: '+d.reset+'\nrover T:139: '+(d.rr||'(not read)');
}
async function poll(){try{S=await (await fetch('/trim/status',{cache:'no-store'})).json();lastOk=Date.now();chip('cN',1,'Page OK');render();}catch(e){chip('cN',0,'Page: no box');}}
setInterval(poll,500);poll();
async function cpoll(){try{const k=await (await fetch('/cam/status',{cache:'no-store'})).json();
 chip('kP',k.paired,k.paired?'Camera paired':'Camera: searching');chip('kF',k.fresh,k.fresh?'Data fresh':'Data STALE');
 chip('kS',k.fresh&&k.strip,k.fresh&&k.strip?'Strip FOUND':'No strip');
 const o=k.off;$('kTxt').textContent=!k.fresh?'No usable camera data (rover would use the gyro only)':(k.strip?('Strip '+(Math.abs(o)<3?'centred':(o>0?'RIGHT ':'LEFT ')+Math.abs(o).toFixed(1)+' px')+', angle '+k.ang.toFixed(1)+' deg'):'No strip in view');
 const w=k.fresh&&k.strip?Math.min(50,Math.abs(o)/60*50):0;const b=$('kBar');b.style.width=w+'%';b.style.left=(o<0?50-w:50)+'%';b.style.background=Math.abs(o)<10?'#4cd07d':'#e0b030';
 $('kNum').textContent=k.rx.toFixed(1)+' msg/s, lost '+k.lost+', longest gap '+k.gap+' ms, round trip '+k.rtt+' ms (max '+k.rttMax+'), signal '+k.rssi+' dBm, brightness '+k.bright+', camera restarts '+k.resets;
 $('kM0').className=k.mode?'':'sel';$('kM1').className=k.mode?'sel':'';
}catch(e){}}
setInterval(cpoll,500);cpoll();
$('kM0').onclick=()=>fetch('/cam/mode?m=0',{cache:'no-store'}).then(cpoll);$('kM1').onclick=()=>fetch('/cam/mode?m=1',{cache:'no-store'}).then(cpoll);
setInterval(()=>{if(run&&!document.hidden)fetch('/trim/alive',{cache:'no-store'}).catch(()=>{});},250);
$('s0').onclick=()=>{si=0;$('s0').className='sel';$('s1').className='';render();};
$('s1').onclick=()=>{si=1;$('s1').className='sel';$('s0').className='';render();};
$('b1').onclick=()=>go('auto');$('b2').onclick=()=>go('fixed');$('b3').onclick=()=>go('verify');
$('bC').onclick=()=>go('custom','&trim='+$('cT').value);
$('stop').onclick=()=>fetch('/trim/stop',{cache:'no-store'}).then(poll);
$('bSave').onclick=()=>{if(confirm('Save the fitted trims in the box memory?'))fetch('/trim/save',{cache:'no-store'}).then(poll);};
$('bDl').onclick=()=>dl('/trim/results.csv','trim_results.csv');$('bLast').onclick=()=>dl('/trim/last.csv','trim_last_run.csv');
$('bClr').onclick=()=>{if(confirm('Clear ALL results?'))fetch('/trim/clear',{cache:'no-store'}).then(poll);};
$('bRate').onclick=()=>fetch('/trim/rate',{cache:'no-store'});
function drv(l,r){seq++;fetch('/drive?l='+l+'&r='+r+'&id='+PID+'&n='+seq,{cache:'no-store'}).catch(()=>{});}
document.querySelectorAll('.pad button').forEach(b=>{const l=b.dataset.l,r=b.dataset.r;
 const start=e=>{e.preventDefault();if(run)return;clearInterval(holdT);drv(l,r);holdT=setInterval(()=>drv(l,r),150);};
 const end=e=>{if(holdT){clearInterval(holdT);holdT=null;drv(0,0);}};
 b.addEventListener('pointerdown',start);['pointerup','pointerleave','pointercancel'].forEach(ev=>b.addEventListener(ev,end));});
document.addEventListener('visibilitychange',()=>{if(document.hidden&&holdT){clearInterval(holdT);holdT=null;drv(0,0);}});
</script></body></html>)rawpage";
