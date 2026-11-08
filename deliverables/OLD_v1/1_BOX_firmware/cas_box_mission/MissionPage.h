#pragma once
// MissionPage - phone dashboard of cas_box_mission ("/"). Style of the drift-trim page.
// Network: status every 0.5 s (mission + log in one round), heartbeat 0.25 s while the mission is active.
static const char kMissionPage[] PROGMEM = R"rawpage(<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>Mission</title><style>
*{box-sizing:border-box}body{margin:0;font-family:system-ui,sans-serif;background:#0f1419;color:#e6e6e6;padding-bottom:110px}
.wrap{max-width:560px;margin:0 auto;padding:10px}h1{font-size:20px;margin:6px 0 8px}
.chips span{display:inline-block;padding:3px 9px;border-radius:12px;margin:2px;font-size:13px;background:#2a2f36}
.ok{background:#1f5f3a!important}.bad{background:#7a2525!important}.rec{background:#a33!important}
.card{background:#1a2027;border-radius:12px;padding:12px;margin:10px 0}
.big{font-size:23px;font-weight:700}.mid{font-size:17px}.small{font-size:13px;color:#9aa4ad}
.q{background:#24324a;border:2px solid #4ea1ff;border-radius:10px;padding:10px;margin-top:10px;font-size:17px}
.nums{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;margin-top:10px}.nums div{background:#141a20;border-radius:8px;padding:8px}
.nums b{display:block;font-size:24px}
button{font-size:17px;padding:13px;border-radius:10px;border:0;background:#2f6fd0;color:#fff;width:100%;margin:5px 0}
button.g{background:#2a8a4a}button.o{background:#3a4656}button:disabled{background:#3a3f46;color:#888}
.step{border-left:4px solid #3a3f46;padding:6px 10px;margin:8px 0}.step.done{border-color:#2fa060}.step.now{border-color:#4ea1ff;background:#18222e}
.step b{font-size:15px}
#stop{position:fixed;left:10px;right:10px;bottom:10px;max-width:540px;margin:0 auto;background:#c62828;font-size:24px;padding:20px;font-weight:700}
details{background:#1a2027;border-radius:12px;padding:10px 12px;margin:10px 0}summary{font-size:17px;font-weight:600}
.pad{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;margin-top:8px}.pad button{padding:24px 0;font-size:26px;background:#3a4656;touch-action:none;user-select:none}
pre{white-space:pre-wrap;font-size:12px;color:#c9d1d9;margin:4px 0}table{width:100%;font-size:13px;border-collapse:collapse}td{padding:4px;border-bottom:1px solid #2a2f36}
input{font-size:16px;padding:6px;width:90px;border-radius:6px;border:0}label{font-size:14px}
</style></head><body><div class="wrap">
<h1>Mission: B &rarr; open sea &middot; Kit 04</h1>
<div class="chips"><span id="cR">Rover</span><span id="cI">IMU</span><span id="cL">LiDAR</span><span id="cB">Battery</span><span id="cC">Camera</span><span id="cG">GPS</span><span id="cLog">Log</span><span id="cN">Page</span></div>

<div class="card"><div class="big" id="state">-</div><div class="small" id="msg"></div>
<div class="q" id="qbox" style="display:none"><div id="qtext"></div><div id="qbtn"></div></div>
<div id="actions"></div>
<div class="nums"><div><span class="small" id="dLab">driven</span><b id="dist">-</b></div><div><span class="small">ahead (LiDAR)</span><b id="front">-</b></div><div><span class="small" id="xLab">pillars</span><b id="x3">-</b></div></div></div>

<div class="card"><b>Steps</b>
<div class="step" id="s1"><b>1 &middot; Leg 1: along the left wall of B</b><div class="small" id="t1">Place the rover at the start, parallel to the B wall, wall on the LEFT. Press Start.</div></div>
<div class="step" id="s2"><b>2 &middot; Right turn to the open area</b><div class="small" id="t2">The rover stops and asks you. You decide: turn here / 1 m more.</div></div>
<div class="step" id="s3"><b>3 &middot; Camera look (optional)</b><div class="small" id="t3">3 pictures: is the strip there? Align on it, or go straight on the gyro.</div></div>
<div class="step" id="s4"><b>4 &middot; Open sea: straight to the end wall</b><div class="small" id="t4">Gyro + trim + crab compensation. Camera check every 2.6 m - if it fails, it just continues.</div></div>
<div class="step" id="s5"><b>5 &middot; Finish: save the log</b><div class="small" id="t5">Then download the log file below and send it to me.</div></div></div>

<div class="card"><b>Live</b><pre id="live">-</pre></div>

<details id="manBox"><summary>Manual drive (hold a button) - always logged</summary>
<div class="small">Touching a button during an autonomous run = you take over. Turns use full power.</div>
<div class="pad"><span></span><button data-l="0.2" data-r="0.2">&#9650;</button><span></span>
<button data-l="-0.3" data-r="0.3">&#10226;</button><button data-l="-0.15" data-r="-0.15">&#9660;</button><button data-l="0.3" data-r="-0.3">&#10227;</button></div></details>

<div class="card"><b>Log</b> <span class="small" id="lgInfo"></span>
<div style="display:flex;gap:8px"><button class="o" id="lgStart">Start log</button><button class="o" id="lgStop">Stop log</button><button class="o" id="lgMark">Mark</button></div>
<table id="files"></table><button class="o" id="lgClr" style="background:#7a2525">Delete all logs</button></div>

<details><summary>Settings</summary>
<label>speed <input id="eSpeed" type="number" step="0.01"></label> <label>ask for the turn after (m) <input id="eT1" type="number" step="0.5"></label><br>
<label>open sea length (m) <input id="eL2" type="number" step="1"></label> <label>stop before end wall (m) <input id="eEnd" type="number" step="0.1"></label><br>
<label>camera px per m <input id="ePx" type="number" step="10"></label> <label>crab prior (deg) <input id="eCrab" type="number" step="0.5"></label><br>
<label><input id="eCam" type="checkbox"> camera may steer (else only logged)</label><br>
<label><input id="eAuto" type="checkbox"> decide alone after the camera look</label><br>
<label><input id="eMode" type="checkbox"> narrow strip (belt) instead of wide tactile strip</label>
<button class="o" id="eSave">Save settings</button></details>
</div>
<button id="stop">&#9632; STOP</button>
<script>
const $=id=>document.getElementById(id);let M=null,G=null,setDone=false;
const PID=Math.floor(Math.random()*1e9);let seq=0,holdT=null;
function chip(id,ok,t,cls){const e=$(id);e.textContent=t;e.className=cls||(ok?'ok':'bad');}
const cmd=c=>fetch('/mis/cmd?c='+c,{cache:'no-store'}).then(poll);
function btn(t,c,cl){return "<button class='"+(cl||'')+"' onclick=\"cmd('"+c+"')\">"+t+"</button>";}
async function dl(n){try{const b=await (await fetch('/log/file?name='+n,{cache:'no-store'})).blob();const a=document.createElement('a');a.href=URL.createObjectURL(b);a.download=n;document.body.appendChild(a);a.click();a.remove();}catch(e){alert('download failed');}}
function render(){if(!M||!G)return;const st=M.st;
 chip('cR',M.rover,'Rover '+(M.rover?'OK':'--'));chip('cI',M.imu,'IMU '+(M.imu?'OK':'--'));chip('cL',M.lidar,'LiDAR '+(M.lidar?'OK':'--'));
 chip('cB',M.batt>11.3,'Battery '+(M.batt>0?M.batt.toFixed(1)+' V':'?'));chip('cC',M.camPaired,M.camPaired?'Camera paired':'Camera --');
 chip('cG',M.gps,'GPS '+M.sats+' sat');chip('cLog',G.active,G.active?('REC '+G.file):'Log off',G.active?'rec':(G.err?'bad':''));
 $('state').textContent=M.state;$('state').style.color=[1,3,7,8].includes(st)?'#4cd07d':([2,6,10].includes(st)?'#4ea1ff':([9,12].includes(st)?'#e0b030':''));
 $('msg').textContent=M.msg||'';
 const q=M.q;$('qbox').style.display=q?'block':'none';$('qtext').textContent=q;
 let qb='';if(st==2)qb=btn('Turn RIGHT here','turn','g')+btn('Drive on 1 m','on1m')+btn('Manual','manual','o');
 if(st==6)qb=(M.lookN>=2?btn('Align on the strip &amp; go','align','g'):'')+btn('Go straight (gyro + crab)','go',M.lookN>=2?'':'g')+btn('Manual','manual','o');
 if(st==10)qb=btn('Finish (save log)','finish','g')+btn('Manual','manual','o');$('qbtn').innerHTML=qb;
 let a='';if(st==0||st==13)a=btn('Start mission (leg 1)','start','g')+btn('Manual drive + log','manual','o')+btn('Start open sea HERE','l2here','o');
 if([1,3,4,5,7,8,9].includes(st))a=btn('Take over manually','manual','o');
 if(st==12)a=btn('Resume','resume','g')+btn('Manual','manual','o')+btn('Finish (save log)','finish','o');
 if(st==11)a=btn('Resume autonomous ('+M.prev+')','resume','g')+btn('Start open sea HERE','l2here')+btn('Finish (save log)','finish','o');
 $('actions').innerHTML=a;if(st==11)$('manBox').open=true;
 const l2=M.leg==2;$('dLab').textContent=l2?'open sea driven':'leg 1 driven';$('dist').textContent=(l2?M.dist2:M.dist).toFixed(1)+' m';
 $('front').textContent=M.front>=9.9?'> 9 m':M.front.toFixed(2)+' m';$('xLab').textContent=l2?'heading error':'pillars';
 $('x3').textContent=l2?(M.err.toFixed(1)+' deg'):M.pillars;
 const done=(n)=>{const e=$('s'+n);e.className='step done';},now=n=>{$('s'+n).className='step now';};
 for(let i=1;i<=5;i++)$('s'+i).className='step';
 if(st==1){now(1);}else if(st==2){done(1);now(2);}else if(st==3||st==4){done(1);now(2);}else if(st==5||st==6||st==7){done(1);done(2);now(3);}
 else if(st==8){done(1);done(2);done(3);now(4);}else if(st==10){[1,2,3,4].forEach(done);now(5);}else if(st==13){[1,2,3,4,5].forEach(done);}
 $('t1').textContent=M.leg>=1&&st!=0?('wall '+(M.wallOk?M.wallD.toFixed(2)+' m (keep '+M.dSet.toFixed(2)+')':'not seen - gyro')+', pillars '+M.pillars+', crab '+M.crab.toFixed(1)+' deg from '+M.crabN+' samples'):'Place the rover at the start, parallel to the B wall, wall on the LEFT. Press Start.';
 $('t3').textContent=M.lookN||st>=6?('strip in '+M.lookN+'/3 pictures, angle '+M.lookAng.toFixed(1)+' deg, offset '+M.lookOff.toFixed(0)+' px'):'3 pictures: is the strip there? Align on it, or go straight on the gyro.';
 $('live').textContent='yaw '+M.yaw+'  target '+M.tgt+'  error '+M.err+' deg  (trim learned '+M.learned+')\n'+
  'ahead: '+M.front.toFixed(2)+' m (LiDAR '+M.lidarF+')'+(M.slope?'  ON SLOPE '+M.pitch+' deg: ramp floor ignored':'')+'\n'+
  'camera: '+(M.camPaired?'paired':'NOT paired')+', snapshots '+M.snapsOk+'/'+M.snaps+' answered, last '+M.snapRtt+' ms, last correction '+M.corr+' deg, next check at '+M.nextCheck+' m\n'+
  'leg 1: '+M.dist+' m, ask at '+M.askAt+' m | open sea: '+M.dist2+' m of '+M.set.l2+' | heap '+M.heap+' kB';
 $('lgInfo').textContent=(G.active?'recording '+G.file+', '+G.recs+' records ('+Math.round(G.recs/4)+' s)':'not recording')+' | '+G.usedKB+'/'+G.totalKB+' kB'+(G.err?' | '+G.err:'');
 let t='';G.files.forEach(f=>{t+='<tr><td>'+f[0]+'</td><td>'+f[1]+' kB</td><td>'+((G.active&&f[0]==G.file)?'(recording)':"<button class='o' style='padding:6px;margin:0' onclick=\"dl('"+f[0]+"')\">download</button>")+'</td></tr>';});$('files').innerHTML=t;
 $('lgStart').disabled=G.active;$('lgStop').disabled=!G.active||(st!=0&&st!=13&&st!=11&&st!=12);$('lgClr').disabled=G.active;
 if(!setDone){const s=M.set;$('eSpeed').value=s.speed;$('eT1').value=s.t1;$('eL2').value=s.l2;$('eEnd').value=s.end;$('ePx').value=s.px;$('eCrab').value=s.crab;$('eCam').checked=!!s.cam;$('eAuto').checked=!!s.auto;$('eMode').checked=!!s.mode;setDone=true;}
}
async function poll(){try{[M,G]=await Promise.all(['/mis/status','/log/status'].map(u=>fetch(u,{cache:'no-store'}).then(r=>r.json())));chip('cN',1,'Page OK');render();}catch(e){chip('cN',0,'Page: no box');}}
setInterval(poll,500);poll();
setInterval(()=>{if(M&&M.st!=0&&M.st!=13&&!document.hidden)fetch('/mis/alive',{cache:'no-store'}).catch(()=>{});},250);
$('stop').onclick=()=>cmd('stop');
$('lgStart').onclick=()=>fetch('/log/start').then(poll);$('lgStop').onclick=()=>fetch('/log/stop').then(poll);$('lgMark').onclick=()=>fetch('/log/mark').then(poll);
$('lgClr').onclick=()=>{if(confirm('Delete ALL logs on the box? Download them first!'))fetch('/log/clear').then(poll);};
$('eSave').onclick=()=>fetch('/mis/set?speed='+$('eSpeed').value+'&t1='+$('eT1').value+'&l2='+$('eL2').value+'&end='+$('eEnd').value+'&px='+$('ePx').value+'&crab='+$('eCrab').value+'&cam='+($('eCam').checked?1:0)+'&auto='+($('eAuto').checked?1:0)+'&mode='+($('eMode').checked?1:0)).then(()=>{setDone=false;poll();});
function drv(l,r){seq++;fetch('/drive?l='+l+'&r='+r+'&id='+PID+'&n='+seq,{cache:'no-store'}).catch(()=>{});}
document.querySelectorAll('.pad button').forEach(b=>{const l=b.dataset.l,r=b.dataset.r;
 const start=e=>{e.preventDefault();clearInterval(holdT);drv(l,r);holdT=setInterval(()=>drv(l,r),150);};
 const end=e=>{if(holdT){clearInterval(holdT);holdT=null;drv(0,0);}};
 b.addEventListener('pointerdown',start);['pointerup','pointerleave','pointercancel'].forEach(ev=>b.addEventListener(ev,end));});
document.addEventListener('visibilitychange',()=>{if(document.hidden&&holdT){clearInterval(holdT);holdT=null;drv(0,0);}});
</script></body></html>)rawpage";
