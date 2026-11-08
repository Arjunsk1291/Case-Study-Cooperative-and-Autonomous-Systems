#pragma once
// FollowPage - phone page of cas_box_drive: strip following (served at "/"). The trim tool is at "/trim".
// Network: status + camera every 0.5 s; heartbeat every 0.25 s only while a run is active.
static const char kFollowPage[] PROGMEM = R"rawpage(<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>Strip follow</title><style>
*{box-sizing:border-box}body{margin:0;font-family:system-ui,sans-serif;background:#0f1419;color:#e6e6e6;padding-bottom:110px}
.wrap{max-width:560px;margin:0 auto;padding:10px}h1{font-size:20px;margin:6px 0 8px}
.chips span{display:inline-block;padding:3px 9px;border-radius:12px;margin:2px;font-size:13px;background:#2a2f36}
.ok{background:#1f5f3a!important}.bad{background:#7a2525!important}
.card{background:#1a2027;border-radius:12px;padding:12px;margin:10px 0}
.big{font-size:24px;font-weight:700}.mid{font-size:17px}.small{font-size:13px;color:#9aa4ad}
.lane{position:relative;height:90px;background:#2a2f36;border-radius:10px;margin:10px 0 4px;overflow:hidden}
.lane .c{position:absolute;left:50%;top:0;bottom:0;width:2px;background:#e6e6e6;opacity:.6}
.lane .s{position:absolute;top:8px;bottom:8px;width:26px;background:#111;border:2px solid #4cd07d;border-radius:4px;transform-origin:50% 50%}
.lr{display:flex;justify-content:space-between;font-size:12px;color:#9aa4ad}
.seg{display:flex;gap:6px}.seg button{flex:1}
button{font-size:17px;padding:13px;border-radius:10px;border:0;background:#2f6fd0;color:#fff;width:100%;margin:5px 0}
.seg button:not(.sel){background:#2a2f36}button:disabled{background:#3a3f46;color:#888}
#stop{position:fixed;left:10px;right:10px;bottom:10px;max-width:540px;margin:0 auto;background:#c62828;font-size:24px;padding:20px;font-weight:700}
.num{font-size:34px;font-weight:700}table{width:100%;border-collapse:collapse;font-size:13px}td,th{padding:4px;border-bottom:1px solid #2a2f36;text-align:left}
details{background:#1a2027;border-radius:12px;padding:10px 12px;margin:10px 0}summary{font-size:17px;font-weight:600}
.pad{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;margin-top:10px}.pad button{padding:22px 0;font-size:24px;background:#3a4656;touch-action:none;user-select:none}
pre{white-space:pre-wrap;font-size:12px;color:#c9d1d9}a{color:#4ea1ff}
</style></head><body><div class="wrap">
<h1>Strip follow &middot; Kit 04</h1>
<div class="chips"><span id="cR">Rover</span><span id="cI">IMU</span><span id="cL">LiDAR</span><span id="cB">Battery</span><span id="cP">Camera</span><span id="cF">Data</span><span id="cN">Page</span></div>

<div class="card"><div class="big" id="ph">-</div><div class="small" id="msg"></div>
<div style="display:flex;gap:10px;margin-top:8px"><div style="flex:1"><div class="small">driven</div><div class="num" id="dist">-</div></div>
<div style="flex:1"><div class="small">wall ahead (LiDAR)</div><div class="num" id="front">-</div></div>
<div style="flex:1"><div class="small">strip seen</div><div class="num" id="seen">-</div></div></div></div>

<div class="card"><b>What the camera sees</b>
<div class="lane"><div class="c"></div><div class="s" id="strip"></div></div>
<div class="lr"><span>&larr; strip LEFT of rover</span><span>centre</span><span>strip RIGHT &rarr;</span></div>
<div class="mid" id="kTxt" style="margin-top:6px">-</div><div class="small" id="kNum"></div>
<div class="small" style="margin-top:8px">Strip width the camera looks for:</div>
<div class="seg"><button id="m0">Wide</button><button id="m1">Narrow (belt)</button></div>
<button id="bZero" style="background:#3a4656">Set centre = rover is ON the strip now</button></div>

<div class="card"><b>Run</b>
<div class="small">Speed</div><div class="seg"><button id="v0">0.15</button><button id="v1" class="sel">0.20</button><button id="v2">0.25</button></div>
<div class="small">Stop before the wall at</div><div class="seg"><button id="w0">0.40 m</button><button id="w1" class="sel">0.50 m</button><button id="w2">0.70 m</button></div>
<div class="small">Never drive more than</div><div class="seg"><button id="x0">2 m</button><button id="x1" class="sel">4 m</button><button id="x2">6 m</button></div>
<button id="bA">1 &middot; Align only (turn onto the strip, no driving)</button>
<button id="bB">2 &middot; Align + follow the strip to the wall</button>
<button id="bF" style="background:#3a4656">Follow only (rover already on the strip)</button>
<button id="bDl" style="background:#3a4656">Download last run log (CSV)</button>
<button id="bRuns" style="background:#3a4656">Download all runs (CSV)</button></div>

<div class="card"><b>Runs</b><table id="tab"></table></div>

<details><summary>Move rover (hold a button)</summary><div class="small">Only while no run is active. Release = gentle stop. Turns use full power.</div>
<div class="pad"><span></span><button data-l="0.2" data-r="0.2">&#9650;</button><span></span>
<button data-l="-0.3" data-r="0.3">&#10226;</button><button data-l="-0.15" data-r="-0.15">&#9660;</button><button data-l="0.3" data-r="-0.3">&#10227;</button></div></details>
<details><summary>Debug</summary><pre id="dbg">-</pre><a href="/trim">Wheel trim tool</a></details>
</div>
<button id="stop">&#9632; STOP</button>
<script>
const $=id=>document.getElementById(id);let F=null,K=null,run=false,spd=0.20,stopM=0.5,maxM=4;
const PID=Math.floor(Math.random()*1e9);let seq=0,holdT=null;
function chip(id,ok,t){const e=$(id);e.textContent=t;e.className=ok?'ok':'bad';}
function seg(ids,i){ids.forEach((d,k)=>$(d).className=k==i?'sel':'');}
async function dl(u,n){try{const b=await (await fetch(u,{cache:'no-store'})).blob();const a=document.createElement('a');a.href=URL.createObjectURL(b);a.download=n;document.body.appendChild(a);a.click();a.remove();}catch(e){alert('download failed');}}
function render(){if(!F||!K)return;run=F.ph>0;
 $('ph').textContent=run?('RUN ('+F.what+'): '+F.phase+' - '+F.t+' s'):'Ready';$('ph').style.color=run?(F.ph==3?'#4cd07d':'#4ea1ff'):'';
 $('msg').textContent=(!run&&F.why?'Last run ended: '+F.why+'. ':'')+(F.msg||'');
 $('dist').textContent=F.dist.toFixed(2)+' m';$('front').textContent=F.front>=9.9?'> 9 m':F.front.toFixed(2)+' m';
 $('front').style.color=F.front<=F.stopM?'#ff7272':(F.front<F.stopM+0.5?'#e0b030':'');$('seen').textContent=run?F.seen+' %':'-';
 const st=K.fresh&&K.strip,o=F.offC;const s=$('strip');s.style.display=st?'block':'none';
 s.style.left='calc('+(50+Math.max(-48,Math.min(48,o/60*50)))+'% - 13px)';s.style.transform='rotate('+K.ang+'deg)';
 s.style.borderColor=Math.abs(o)<8?'#4cd07d':'#e0b030';
 $('kTxt').textContent=!K.fresh?'NO camera data (rover would use the gyro only)':(st?('Strip '+(Math.abs(o)<3?'centred':(o>0?'RIGHT ':'LEFT ')+Math.abs(o).toFixed(1)+' px')+', angle '+K.ang.toFixed(1)+' deg ('+K.rows+'/4 rows)'):'No strip in view');
 $('kNum').textContent=K.rx.toFixed(1)+' msg/s, lost '+K.lost+', round trip '+K.rtt+' ms, signal '+K.rssi+' dBm, brightness '+K.bright+' | centre = '+F.zero+' px';
 seg(['m0','m1'],K.mode?1:0);
 const can=!run&&D&&D.rover&&D.imu&&D.lidar;
 ['bA','bB','bZero','bDl','bRuns'].forEach(b=>$(b).disabled=!can&&b!='bDl'&&b!='bRuns'||run);$('bF').disabled=!can;
 $('bA').disabled=$('bA').disabled||!st;$('bB').disabled=$('bB').disabled||!st;$('bZero').disabled=run||!st;
 let t='<tr><th>#</th><th>what</th><th>m</th><th>s</th><th>seen</th><th>max off</th><th>wall</th><th>end</th></tr>';
 F.runs.slice().reverse().forEach(r=>{t+='<tr><td>'+r[0]+'</td><td>'+r[1]+'</td><td>'+r[2].toFixed(2)+'</td><td>'+r[3].toFixed(1)+'</td><td>'+r[4]+'%</td><td>'+r[5].toFixed(1)+'</td><td>'+(r[6]===null?'-':r[6].toFixed(2))+'</td><td>'+r[7]+'</td></tr>';});
 $('tab').innerHTML=t;
 $('dbg').textContent='yaw '+F.yaw+'  target '+F.tgt+'  heading-hold: '+(F.ds||'ok')+'\ncamera raw offset '+K.off+' px, centre '+F.zero+' px, angle '+K.ang+' deg, rows '+K.rows+'\nlongest camera gap '+K.gap+' ms, max round trip '+K.rttMax+' ms, camera restarts '+K.resets+(D?'\nbattery '+D.batt+' V, rover rtt '+D.dbg.rtt+' ms ok '+D.dbg.ok+' fail '+D.dbg.fail+'\nheap '+D.dbg.heap+' kB (min '+D.dbg.minheap+'), up '+D.dbg.up+' s, last reset: '+D.dbg.reset:'');
}
let D=null;
async function poll(){try{[F,K,D]=await Promise.all(['/follow/status','/cam/status','/trim/status'].map(u=>fetch(u,{cache:'no-store'}).then(r=>r.json())));
 chip('cN',1,'Page OK');chip('cR',D.rover,'Rover '+(D.rover?'OK':'--'));chip('cI',D.imu,'IMU '+(D.imu?'OK':'--'));chip('cL',D.lidar,'LiDAR '+(D.lidar?'OK':'--'));
 chip('cB',D.batt>11.3,'Battery '+(D.batt>0?D.batt.toFixed(1)+' V':'?'));chip('cP',K.paired,K.paired?'Camera paired':'Camera searching');chip('cF',K.fresh,K.fresh?'Data fresh':'Data STALE');render();
}catch(e){chip('cN',0,'Page: no box');}}
setInterval(poll,500);poll();
setInterval(()=>{if(run&&!document.hidden)fetch('/follow/alive',{cache:'no-store'}).catch(()=>{});},250);
const go=w=>fetch('/follow/start?what='+w+'&spd='+spd+'&stop='+stopM+'&max='+maxM,{cache:'no-store'}).then(()=>{run=true;poll();});
$('bA').onclick=()=>go(0);$('bB').onclick=()=>go(2);$('bF').onclick=()=>go(1);
$('stop').onclick=()=>fetch('/follow/stop',{cache:'no-store'}).then(poll);
$('bZero').onclick=()=>fetch('/follow/zero',{cache:'no-store'}).then(poll);
$('m0').onclick=()=>fetch('/cam/mode?m=0',{cache:'no-store'}).then(poll);$('m1').onclick=()=>fetch('/cam/mode?m=1',{cache:'no-store'}).then(poll);
[['v0',0.15],['v1',0.20],['v2',0.25]].forEach(([b,v],i,a)=>$(b).onclick=()=>{spd=v;seg(a.map(x=>x[0]),i);});
[['w0',0.4],['w1',0.5],['w2',0.7]].forEach(([b,v],i,a)=>$(b).onclick=()=>{stopM=v;seg(a.map(x=>x[0]),i);});
[['x0',2],['x1',4],['x2',6]].forEach(([b,v],i,a)=>$(b).onclick=()=>{maxM=v;seg(a.map(x=>x[0]),i);});
$('bDl').onclick=()=>dl('/follow/log.csv','follow_last_run.csv');$('bRuns').onclick=()=>dl('/follow/runs.csv','follow_runs.csv');
function drv(l,r){seq++;fetch('/drive?l='+l+'&r='+r+'&id='+PID+'&n='+seq,{cache:'no-store'}).catch(()=>{});}
document.querySelectorAll('.pad button').forEach(b=>{const l=b.dataset.l,r=b.dataset.r;
 const start=e=>{e.preventDefault();if(run)return;clearInterval(holdT);drv(l,r);holdT=setInterval(()=>drv(l,r),150);};
 const end=e=>{if(holdT){clearInterval(holdT);holdT=null;drv(0,0);}};
 b.addEventListener('pointerdown',start);['pointerup','pointerleave','pointercancel'].forEach(ev=>b.addEventListener(ev,end));});
document.addEventListener('visibilitychange',()=>{if(document.hidden&&holdT){clearInterval(holdT);holdT=null;drv(0,0);}});
</script></body></html>)rawpage";
