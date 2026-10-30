#include "WebHMI.h"

// ---------------------------------------------------------------------------------------------
// The web page. %TITLE% and %MAXSPD% are filled in by _buildPage().
// ---------------------------------------------------------------------------------------------
static const char kPage[] PROGMEM = R"rawpage(<!doctype html><html><head><meta charset='utf-8'>
<meta name='viewport' content='width=device-width, initial-scale=1'><title>%TITLE%</title>
<style>
:root{--bg:#0b0f17;--card:#121a27;--muted:#9aa4b2;--text:#e7edf5;--acc:#4ea1ff;--warn:#ffb020;}
html,body{margin:0;background:var(--bg);color:var(--text);font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;}
.wrap{max-width:980px;margin:0 auto;padding:16px;}
.top{display:flex;align-items:center;justify-content:space-between;gap:12px;}
.title{font-size:18px;font-weight:700;}
.pill{font-size:12px;color:var(--muted);padding:6px 10px;border:1px solid rgba(255,255,255,.08);border-radius:999px;}
.card{background:var(--card);border:1px solid rgba(255,255,255,.06);border-radius:16px;padding:14px;margin-top:12px;}
.label{font-size:12px;color:var(--muted);}
.value{font-size:16px;font-weight:650;}
.sectionTitle{font-size:13px;color:var(--muted);margin:0 0 8px;}
.hidden{display:none;}
button{appearance:none;border:0;border-radius:14px;font-weight:650;cursor:pointer;color:var(--text);
 background:rgba(255,255,255,.06);border:1px solid rgba(255,255,255,.09);}
.btnTiny{padding:10px 12px;font-size:13px;}
.stop{background:linear-gradient(180deg,rgba(255,90,95,.95),rgba(255,90,95,.75));color:#190406;}
.safetyBar{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin-top:10px}.safetyBar button{padding:11px 14px}.safetyState{font-size:12px;color:var(--muted)}
.pad{display:grid;grid-template-columns:repeat(3,72px);grid-template-rows:repeat(3,72px);gap:10px;
 justify-content:center;margin:6px 0 10px;touch-action:none;user-select:none;-webkit-user-select:none;}
.pad button{font-size:24px;touch-action:none;}
.pad button.on{outline:3px solid var(--acc);}
input[type=range]{width:100%;}
.table{width:100%;border-collapse:collapse;font-size:13px;}
.table td{padding:8px 7px;border-bottom:1px solid rgba(255,255,255,.06);}
.tiles{display:grid;grid-template-columns:repeat(auto-fit,minmax(135px,1fr));gap:8px;margin-top:10px}
.tile{padding:10px;border-radius:12px;background:#0b111b;border:1px solid rgba(255,255,255,.08)}
.tile .label{display:block;margin-bottom:5px}.tile .value{font-size:16px;overflow-wrap:anywhere}
.issue{padding:10px;border-radius:10px;margin-top:7px;background:rgba(255,176,32,.09);border:1px solid rgba(255,176,32,.2)}
.okIssue{background:rgba(85,214,158,.08);border-color:rgba(85,214,158,.2)}
.usStatus{font-size:11px;color:var(--muted)}.usGood{color:#55d69e}.usWarn{color:var(--warn)}.usBad{color:#ff7378}
.k{color:var(--muted);}
canvas{display:block;margin:0 auto;max-width:100%;}
</style></head><body><div class='wrap'>
<div class='top'><div class='title'>%TITLE%</div><div class='pill' id='conn'>-</div>
 <div style='display:flex;gap:8px;align-items:center'><button id='emergencyStop' class='btnTiny stop'>STOP ROVER / E-STOP</button><button id='devBtn' class='btnTiny'>Dev</button></div></div>
 <div class='safetyBar'><button id='clearStop' class='btnTiny' disabled>Clear stop latch</button><span class='safetyState' id='safetyState'>Drive interlock: checking</span></div>
<div class='card'><div class='label'>State</div><div class='value' id='state'>-</div></div>
<div class='card'><div class='sectionTitle'>Live troubleshooting</div>
 <div class='label'>Use this panel to tell whether the box is sensing, connected to the rover, commanding motion, or recovering from suspected no movement. Motion is inferred from LiDAR scene change because this rover has no wheel encoders.</div>
 <div class='tiles'>
  <div class='tile'><span class='label'>Rover link</span><span class='value' id='diagLink'>Checking…</span></div>
  <div class='tile'><span class='label'>Drive command L / R</span><span class='value' id='diagCmd'>0.00 / 0.00</span></div>
  <div class='tile'><span class='label'>Adaptive forward speed</span><span class='value' id='diagSpeed'>—</span></div>
  <div class='tile'><span class='label'>Self-corrections</span><span class='value' id='diagRecoveries'>0</span></div>
  <div class='tile'><span class='label'>LiDAR</span><span class='value' id='diagLidar'>Checking…</span></div>
  <div class='tile'><span class='label'>Front clearance</span><span class='value' id='diagFront'>—</span></div>
 </div>
 <div id='diagIssue' class='issue'>Waiting for live diagnostics…</div>
 <div class='label' id='diagAdvice' style='margin-top:8px'>If the rover does not move, first check rover link/IP, motor battery, wheel obstruction and whether the rover itself is powered.</div>
</div>
<div class='card'><div class='sectionTitle'>Landmark-guided washroom navigation (experimental)</div><div class='label'>Uses recorded landmark references in route order. Unknown localization, stale sensors, or insufficient doorway clearance must stop motion; it must not explore arbitrary open sectors. Keep a spotter and physical stop access.</div><div style='display:flex;gap:8px;margin-top:10px;flex-wrap:wrap'><button id='autoBtn' class='btnTiny'>Enable autonomy (OFF)</button><button id='autoStop' class='btnTiny stop'>Disable autonomy / STOP</button></div><div id='autoStatus' class='label' style='margin-top:8px'>Autonomy OFF · route localization required</div></div>
<div class='card hidden' id='driveCard'><div class='sectionTitle'>Drive (hold a key)</div>
 <div class='pad'>
  <span></span><button data-d='f'>&#9650;</button><span></span>
  <button data-d='l'>&#9664;</button><button class='stop' data-d='s'>&#9632;</button><button data-d='r'>&#9654;</button>
  <span></span><button data-d='b'>&#9660;</button><span></span>
 </div>
 <div class='label'>Speed <span id='spdVal'></span></div><input type='range' id='spd' min='0.05' step='0.05'></div>
<div class='card' id='investigationCard'>
 <div class='sectionTitle'>Investigation mode · route survey</div>
 <div class='label'>Guided sequence: START scan → travel with continuous logging → arrive and scan each waypoint → final destination scan → export one JSON. Keep autonomy OFF. The drive pad stays available during travel. No encoder is assumed.</div>
 <div style='display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin-top:12px'>
  <label for='invLabel' class='label'>Current / next location</label><input id='invLabel' value='START' maxlength='32' style='min-width:120px;flex:1;padding:10px;border-radius:8px;background:#0b0f17;color:#e7edf5;border:1px solid rgba(255,255,255,.15)'>
  <label for='invDuration' class='label'>Scan seconds</label><select id='invDuration' style='padding:10px;border-radius:8px;background:#0b0f17;color:#e7edf5;border:1px solid rgba(255,255,255,.15)'><option value='10'>10 s</option><option value='15' selected>15 s</option><option value='20'>20 s</option></select>
 </div>
 <div style='display:flex;gap:8px;flex-wrap:wrap;margin-top:12px'>
  <button id='invStart' class='btnTiny'>Start · scan START</button>
  <button id='invNext' class='btnTiny' disabled>Next · begin travel</button>
  <button id='invLast' class='btnTiny stop' disabled>Last · finish at destination</button>
  <button id='invDownload' class='btnTiny' disabled>Download complete JSON</button>
 </div>
 <div id='invPhase' style='font-size:1.05rem;font-weight:700;margin-top:14px'>Ready to begin</div>
 <div id='invTimerWrap' style='display:none;margin-top:8px'><div id='invTimerText' class='label'>Capturing…</div><div style='height:10px;background:#222b39;border-radius:99px;overflow:hidden;margin-top:6px'><div id='invTimerBar' style='height:100%;width:100%;background:#4ea1ff;transition:width .2s linear'></div></div></div>
 <div style='display:flex;gap:8px;flex-wrap:wrap;margin-top:12px'><div class='tile'><span class='label'>Route samples</span><span class='value' id='invSamples'>0</span></div><div class='tile'><span class='label'>Locations</span><span class='value' id='invLocations'>0</span></div><div class='tile'><span class='label'>Elapsed</span><span class='value' id='invElapsed'>00:00</span></div></div>
 <div id='invInstruction' class='label' style='margin-top:10px'>Place rover at START. Keep hands clear of wheels. During each scan, keep still for a few seconds, then rotate the rover slowly by hand using the drive controls only if safe; the log captures each changing LiDAR/IMU frame.</div>
 <div id='invStatus' class='label' style='margin-top:8px'>Investigation idle</div>
 <div class='label' style='margin-top:8px'>Safety and data limits: do not let the rover autonomously spin. This workflow records sensor frames and travel snapshots, not reliable distance or a finished SLAM map. LiDAR scan matching can help estimate relative motion, but it cannot guarantee return to the same place until scan matching/localization is implemented and validated.</div>
</div>
<div class='card'><div class='sectionTitle'>Indoor scan / route data logger</div>
 <div class='label'>Records sensor snapshots in this browser at the dashboard update rate. Mark physical locations such as A, corridor turn, and B. Download the CSV before closing or refreshing this page.</div>
 <div style='display:flex;gap:8px;flex-wrap:wrap;margin-top:10px'>
  <button id='logStart' class='btnTiny'>Start recording</button>
  <button id='logMark' class='btnTiny' disabled>Mark waypoint</button>
  <button id='logStop' class='btnTiny' disabled>Stop</button>
  <button id='logDownload' class='btnTiny' disabled>Download CSV</button>
  <button id='diagDownload' class='btnTiny'>Download diagnostic JSON</button>
 </div>
 <div style='display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin-top:10px'>
  <label for='wpLabel' class='label'>Waypoint label</label><input id='wpLabel' value='A' maxlength='24' style='min-width:90px;flex:1;padding:9px;border-radius:8px;background:#0b0f17;color:#e7edf5;border:1px solid rgba(255,255,255,.15)'>
  <span id='logStatus' class='label'>Not recording</span>
 </div>
 <div class='label' style='margin-top:8px'>Log is saved in browser memory until downloaded. This is sensor logging, not an indoor SLAM/map-coordinate solver.</div>
 <label class='label' for='operatorNotes' style='display:block;margin-top:10px'>Operator notes (optional; included in diagnostic JSON)</label><textarea id='operatorNotes' rows='3' style='box-sizing:border-box;width:100%;padding:9px;border-radius:8px;background:#0b0f17;color:#e7edf5;border:1px solid rgba(255,255,255,.15)' placeholder='What did the rover do physically? Where was the obstacle?'></textarea>
</div>
<div class='card'><div class='sectionTitle'>Surroundings (LiDAR top view, front = up, rings 1 m)</div>
 <canvas id='envCv' width='280' height='280'></canvas><div class='label' id='lidarInfo'>-</div>
 <table class='table' id='usTable'></table></div>
<div class='card'><div class='sectionTitle'>IMU / GPS</div><table class='table' id='navTable'></table></div>
<div id='devRoot' class='hidden'>
 <div class='card'><div class='sectionTitle'>Values</div><table class='table' id='valTable'></table></div>
 <div class='card'><div class='sectionTitle'>Loop time (box)</div>
  <table class='table'><tr><td class='k'>min / avg / max</td><td id='cycle'>-</td></tr></table></div>
</div></div>
<script>
const MAX_SPEED=%MAXSPD%, ENV_MAX_CM=300;
const $=id=>document.getElementById(id);
const esc=s=>(''+s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');

// ---- "Dev" view toggle
let dev=false;
$('emergencyStop').onclick=async()=>{
  $('emergencyStop').disabled=true;
  try {
    const r=await fetch('/safety/stop',{method:'POST',cache:'no-store'});
    $('safetyState').textContent=r.ok?'STOP LATCHED · drive commands blocked':'STOP REQUEST FAILED · connection unavailable';
    $('clearStop').disabled=!r.ok;
  } catch(e) { $('safetyState').textContent='STOP REQUEST FAILED · connection unavailable'; }
  $('emergencyStop').disabled=false;
};
$('clearStop').onclick=async()=>{
  if(!window.confirm('Clear the stop latch? The rover will remain stationary until a new drive command is made.'))return;
  try {
    const r=await fetch('/safety/clear',{method:'POST',cache:'no-store'});
    if(r.ok){$('safetyState').textContent='Stop latch cleared · rover remains stationary';$('clearStop').disabled=true;}
    else $('safetyState').textContent='Could not clear stop latch';
  } catch(e){$('safetyState').textContent='Could not clear stop latch';}
};
$('devBtn').onclick=()=>{dev=!dev;$('devRoot').classList.toggle('hidden',!dev);$('devBtn').textContent=dev?'Hide':'Dev';};

// ---- drive pad: send while a key is held (every 150 ms), stop on release / hide / blur
let drv=null,drvTimer=null,drvBtn=null;
// Requests can overtake each other on Wi-Fi. Each command carries this page's random id and a
// rising number; the box ignores anything older than the last command it accepted.
const PAGE_ID=Math.floor(Math.random()*4e9)+1;let seq=0;
function drvSend(l,r){seq++;fetch('/drive?l='+l.toFixed(2)+'&r='+r.toFixed(2)+'&id='+PAGE_ID+'&n='+seq).catch(()=>{});}
function drvClear(){clearInterval(drvTimer);drvTimer=null;drv=null;if(drvBtn){drvBtn.classList.remove('on');drvBtn=null;}}
function drvStop(){drvClear();drvSend(0,0);}
let autoOn=false;
async function setAuto(on){
  try{await fetch('/autonomy?on='+(on?'1':'0'),{cache:'no-store'});autoOn=on;}
  catch(e){autoOn=false;}
  $('autoBtn').textContent=autoOn?'Autonomy ON':'Enable autonomy (OFF)';
  $('autoStatus').textContent=autoOn?'AUTONOMY ON · scan-and-wander · bounded legs':'Autonomy OFF · manual control available';
  if(autoOn)drvStop();
  if(typeof invUi==='function')invUi();
}
$('autoBtn').onclick=()=>setAuto(!autoOn);
$('autoStop').onclick=()=>setAuto(false);
function drvStart(b){
  if(autoOn){if(b.dataset.d==='s')setAuto(false);return;}
  if(b.dataset.d==='s'){drvStop();return;}
  const s=parseFloat($('spd').value);
  const m={f:[s,s],b:[-s,-s],l:[-s,s],r:[s,-s]}[b.dataset.d];
  drvClear();  // switching keys: no stop in between, the new command follows directly
  drv=m;drvBtn=b;b.classList.add('on');drvSend(m[0],m[1]);
  drvTimer=setInterval(()=>drvSend(drv[0],drv[1]),150);
}
if(MAX_SPEED>0){
  $('driveCard').classList.remove('hidden');
  const sp=$('spd');sp.max=MAX_SPEED;sp.value=MAX_SPEED;
  const show=()=>{$('spdVal').textContent=parseFloat(sp.value).toFixed(2);};
  sp.addEventListener('input',show);show();
  document.querySelectorAll('.pad button').forEach(b=>{
    b.addEventListener('pointerdown',e=>{e.preventDefault();b.setPointerCapture(e.pointerId);drvStart(b);});
    ['pointerup','pointercancel','lostpointercapture'].forEach(ev=>b.addEventListener(ev,()=>{if(drvBtn===b)drvStop();}));
    b.addEventListener('contextmenu',e=>e.preventDefault());
  });
  document.addEventListener('visibilitychange',()=>{if(document.hidden&&drvBtn)drvStop();});
  window.addEventListener('blur',()=>{if(drvBtn)drvStop();});
}

// ---- LiDAR top view: line through neighbouring points, broken at "no return" and at jumps
function drawEnv(s){
  const cv=$('envCv'),g=cv.getContext('2d'),W=cv.width,c=W/2,k=(c-12)/ENV_MAX_CM;
  g.clearRect(0,0,W,W);
  g.strokeStyle='rgba(255,255,255,.12)';g.lineWidth=1;
  for(let m=100;m<=ENV_MAX_CM;m+=100){g.beginPath();g.arc(c,c,m*k,0,2*Math.PI);g.stroke();}
  const d=(s.lidar&&s.lidar.d)||[],N=d.length,st=360/Math.max(N,1);
  const P=d.map((cm,i)=>{if(!cm||cm>ENV_MAX_CM)return null;const a=(i*st+st/2)*Math.PI/180;
    return [c+cm*k*Math.sin(a),c-cm*k*Math.cos(a),cm];});
  const near=(p,q)=>p&&q&&Math.abs(p[2]-q[2])<Math.max(30,0.25*Math.min(p[2],q[2]));
  g.strokeStyle='#4ea1ff';g.fillStyle='#4ea1ff';g.lineWidth=3;g.lineCap='round';
  for(let i=0;i<N;i++){const p=P[i];if(!p)continue;const q=P[(i+1)%N],o=P[(i+N-1)%N];
    if(near(p,q)){g.beginPath();g.moveTo(p[0],p[1]);g.lineTo(q[0],q[1]);g.stroke();}
    else if(!near(o,p)){g.beginPath();g.arc(p[0],p[1],2.5,0,2*Math.PI);g.fill();}}
  g.fillStyle='#e7edf5';g.fillRect(c-9,c-12,18,24);  // box
  g.fillStyle='#ffb020';g.beginPath();g.moveTo(c,c-20);g.lineTo(c-6,c-12);g.lineTo(c+6,c-12);g.fill();  // front
}

// ---- sensor cards
function renderSensors(s){
  if(!s)return;
  drawEnv(s);
  const L=s.lidar||{};
  $('lidarInfo').textContent=L.ok?('LiDAR '+L.pps+' packets/s, checksum errors '+L.crc+' %'):'LiDAR: no data';
  const us=s.us||[],raw=s.usRaw||[],filtered=s.usFiltered||[],valid=s.usValid||[],age=s.usAgeMs||[],timeouts=s.usTimeouts||[];
  const names=['Front','Left','Right','Back','Front angled'];
  let usHtml='<tr><td class="k">Sensor</td><td class="k">Raw</td><td class="k">Median</td><td class="k">Status</td></tr>';
  usHtml+=names.map((n,i)=>{
    const isValid=valid[i]===true, rawText=isValid&&raw[i]>=0?raw[i]+' cm':'NO ECHO';
    const filtText=filtered[i]===undefined||filtered[i]>=s.usMax?'-':filtered[i]+' cm';
    const ageText=age[i]===undefined||age[i]<0?'never':(age[i]<1000?age[i]+' ms':(age[i]/1000).toFixed(1)+' s');
    const status=isValid?`<span class="usGood">Fresh</span>`:`<span class="usWarn">Timeout</span>`;
    return `<tr><td class="k">${n}</td><td>${rawText}</td><td>${filtText}</td><td>${status}<div class="usStatus">last good: ${ageText} · timeouts: ${timeouts[i]||0}</div></td></tr>`;
  }).join('');
  $('usTable').innerHTML=usHtml;
  const I=s.imu||{},G=s.gps||{};
  // ISO 8855: roll + = right side down, pitch + = front down, yaw + = turning left (0 = north)
  let t=I.ok?`<tr><td class='k'>Roll / pitch / yaw (ISO 8855)</td><td>${I.roll}&deg; / ${I.pitch}&deg; / ${I.yaw}&deg;</td></tr>`
    +`<tr><td class='k'>IMU calibration sys/gyr/acc/mag</td><td>${(I.cal||[]).join(' / ')} (3 = good)</td></tr>`
    :`<tr><td class='k'>IMU</td><td>not found</td></tr>`;
  t+=G.chars?`<tr><td class='k'>GPS</td><td>${G.fix?'fix':'no fix'}, ${G.sats} satellites, HDOP ${G.hdop}</td></tr>`
    :`<tr><td class='k'>GPS</td><td>no data from the module</td></tr>`;
  if(G.fix)t+=`<tr><td class='k'>Latitude / longitude</td><td>${G.lat}, ${G.lon}</td></tr>`;
  $('navTable').innerHTML=t;
}

// ---- browser-side sensor logger: CSV download; no SD card or ESP32 RAM buffer needed
let logging=false, logRows=[], logStartMs=0, latestSnapshot=null, logSampleCount=0;
const usNames=['front','left','right','back','front_angled'];
function csvCell(v){
  if(v===null||v===undefined)return '';
  const s=String(v);
  return /[",\\r\\n]/.test(s)?'"'+s.replace(/"/g,'""')+'"':s;
}
const lidarHeaders=Array.from({length:90},(_,i)=>'lidar_cm_bin_'+i);
const logHeaders=['record_type','elapsed_ms','wall_time_iso','waypoint_label','state',
 'us_front_raw_cm','us_front_median_cm','us_front_valid','us_front_age_ms','us_front_timeouts',
 'us_left_raw_cm','us_left_median_cm','us_left_valid','us_left_age_ms','us_left_timeouts',
 'us_right_raw_cm','us_right_median_cm','us_right_valid','us_right_age_ms','us_right_timeouts',
 'us_back_raw_cm','us_back_median_cm','us_back_valid','us_back_age_ms','us_back_timeouts',
 'us_front_angled_raw_cm','us_front_angled_median_cm','us_front_angled_valid','us_front_angled_age_ms','us_front_angled_timeouts',
 'imu_ok','imu_yaw_deg','imu_pitch_deg','imu_roll_deg','imu_cal_sys','imu_cal_gyro','imu_cal_accel','imu_cal_mag',
 'gps_fix','gps_satellites','gps_hdop','gps_lat','gps_lon','lidar_ok','lidar_packets_per_sec','lidar_crc_error_pct',...lidarHeaders];
function rowFor(type,label,s){
  const now=new Date(),us=s.us||[],raw=s.usRaw||[],filt=s.usFiltered||[],valid=s.usValid||[],age=s.usAgeMs||[],timeouts=s.usTimeouts||[];
  const row=[type,Math.max(0,performance.now()-logStartMs).toFixed(0),now.toISOString(),label||'',latestSnapshot&&latestSnapshot.state||''];
  for(let i=0;i<5;i++)row.push(raw[i]===undefined?'':raw[i],filt[i]===undefined?'':filt[i],valid[i]===true?1:0,age[i]===undefined?'':age[i],timeouts[i]===undefined?'':timeouts[i]);
  const I=s.imu||{},cal=I.cal||[],G=s.gps||{},L=s.lidar||{};
  row.push(I.ok?1:0,I.yaw??'',I.pitch??'',I.roll??'',cal[0]??'',cal[1]??'',cal[2]??'',cal[3]??'',
    G.fix?1:0,G.sats??'',G.hdop??'',G.lat??'',G.lon??'',L.ok?1:0,L.pps??'',L.crc??'');
  const d=L.d||[];for(let i=0;i<90;i++)row.push(d[i]===undefined?'':d[i]);
  return row;
}
function updateLogUi(){
  $('logStart').disabled=logging;
  $('logMark').disabled=!logging||!latestSnapshot||!latestSnapshot.sensors;
  $('logStop').disabled=!logging;
  $('logDownload').disabled=logRows.length===0;
  $('logStatus').textContent=logging?('Recording · '+logSampleCount+' samples'):(logRows.length?('Stopped · '+logSampleCount+' samples'):'Not recording');
}
$('logStart').onclick=()=>{
  if(!latestSnapshot||!latestSnapshot.sensors){$('logStatus').textContent='Waiting for sensor data';return;}
  logRows=[];logSampleCount=0;logStartMs=performance.now();logging=true;
  logRows.push(logHeaders.map(csvCell).join(','));
  logRows.push(rowFor('START','',latestSnapshot.sensors).map(csvCell).join(','));
  updateLogUi();
};
$('logMark').onclick=()=>{
  if(!logging||!latestSnapshot||!latestSnapshot.sensors)return;
  const label=$('wpLabel').value.trim()||'UNLABELLED';
  logRows.push(rowFor('WAYPOINT',label,latestSnapshot.sensors).map(csvCell).join(','));
  $('logStatus').textContent='Marked '+label+' · '+logSampleCount+' samples';
};
$('logStop').onclick=()=>{
  if(!logging)return;
  if(latestSnapshot&&latestSnapshot.sensors)logRows.push(rowFor('STOP','',latestSnapshot.sensors).map(csvCell).join(','));
  logging=false;updateLogUi();
};
$('logDownload').onclick=()=>{
  if(!logRows.length)return;
  const blob=new Blob(['\ufeff'+logRows.join('\r\n')],{type:'text/csv;charset=utf-8;'});
  const url=URL.createObjectURL(blob),a=document.createElement('a');
  a.href=url;a.download='CAS_B3_R04_indoor_sensors_'+new Date().toISOString().replace(/[:.]/g,'-')+'.csv';
  document.body.appendChild(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(url),1000);
};
// Guided investigation recorder. Dashboard polls at 4 Hz; every received sensor frame is stored
// during both waypoint scans and travel. Browser memory is used so no unbounded ESP32 heap buffer
// is created. Sensor and rover-link subsystems continue to run in their existing FreeRTOS tasks.
let investigationActive=false, investigationStartIso=null, investigationEvents=[], investigationSamples=0;
let invPhase='idle', invTimer=null, invCaptureEndsAt=0, invCaptureDurationMs=15000, invLocationCount=0, invLastEventMs=0;
function invElapsedMs(){return investigationStartIso?Math.max(0,Date.now()-Date.parse(investigationStartIso)):0;}
function invFmt(ms){const s=Math.floor(ms/1000);return String(Math.floor(s/60)).padStart(2,'0')+':'+String(s%60).padStart(2,'0');}
function invSetPhase(phase,title,instruction){invPhase=phase;$('invPhase').textContent=title;$('invInstruction').textContent=instruction||'';invUi();}
function invUi(){
  const ready=!!(latestSnapshot&&latestSnapshot.sensors),active=investigationActive;
  $('invStart').disabled=active||!ready||autoOn||invPhase==='done';
  $('invNext').disabled=!active||!['capture_done','travel'].includes(invPhase);
  $('invLast').disabled=!active||invPhase!=='travel';
  $('invDownload').disabled=investigationEvents.length===0;
  $('invSamples').textContent=String(investigationSamples);
  $('invLocations').textContent=String(invLocationCount);
  $('invElapsed').textContent=invFmt(invElapsedMs());
}
function invEvent(type,label,extra={}){
  if(!latestSnapshot||!latestSnapshot.sensors)return false;
  const now=Date.now();
  investigationEvents.push({type,label:label||'',captured_at:new Date(now).toISOString(),elapsed_ms:invElapsedMs(),
    delta_since_previous_event_ms:invLastEventMs?now-invLastEventMs:0,phase:invPhase,
    dashboard_snapshot_received_at:new Date(latestSnapshot.receivedAt||now).toISOString(),
    dashboard_snapshot_age_ms:Math.max(0,now-(latestSnapshot.receivedAt||now)),
    dashboard_state:latestSnapshot.state||null,values:Object.fromEntries((latestSnapshot.values||[]).map(v=>[v.name,v.value])),
    cycle:latestSnapshot.cycle||null,sensors:latestSnapshot.sensors,...extra});
  invLastEventMs=now;investigationSamples++;invUi();return true;
}
function invSample(type){
  if(!investigationActive||!latestSnapshot||!latestSnapshot.sensors)return;
  const now=Date.now();
  investigationEvents.push({type,label:$('invLabel').value.trim()||'',captured_at:new Date(now).toISOString(),elapsed_ms:invElapsedMs(),
    phase:invPhase,dashboard_snapshot_received_at:new Date(latestSnapshot.receivedAt||now).toISOString(),
    dashboard_snapshot_age_ms:Math.max(0,now-(latestSnapshot.receivedAt||now)),dashboard_state:latestSnapshot.state||null,
    values:Object.fromEntries((latestSnapshot.values||[]).map(v=>[v.name,v.value])),cycle:latestSnapshot.cycle||null,sensors:latestSnapshot.sensors});
  investigationSamples++;
}
function invStopTimer(){if(invTimer){clearInterval(invTimer);invTimer=null;} $('invTimerWrap').style.display='none';}
function invBeginCapture(label,type){
  if(!investigationActive||!latestSnapshot||!latestSnapshot.sensors)return;
  invStopTimer();invCaptureDurationMs=Number($('invDuration').value||15)*1000;invCaptureEndsAt=Date.now()+invCaptureDurationMs;
  invSetPhase('capturing','Capturing location: '+label,'Capture is in progress. Keep the rover in a safe, clear area. For a useful 360° data set, pause first, then slowly rotate it only with the drive controls and a spotter; stop immediately if anything approaches.');
  invEvent(type,label,{capture_duration_ms:invCaptureDurationMs,capture_role:type==='START_SCAN'?'start':'waypoint',operator_prompt:'A scan is a time series, not a single frame.'});
  $('invTimerWrap').style.display='block';
  const tick=()=>{const left=Math.max(0,invCaptureEndsAt-Date.now()),pct=100*left/invCaptureDurationMs;
    $('invTimerText').textContent='Capture in progress · '+(left/1000).toFixed(1)+' s remaining';$('invTimerBar').style.width=pct+'%';
    if(left<=0){invStopTimer();invEvent('SCAN_COMPLETE',label,{capture_duration_ms:invCaptureDurationMs,scan_sample_count:investigationEvents.filter(e=>e.label===label&&(e.type==='SCAN_SAMPLE')).length});
      invLocationCount++;
      if(type==='END_SCAN'){
        invEvent('INVESTIGATION_STOP','END',{operator_instruction:'Final destination scan completed; arrival and label were operator supplied.'});
        investigationActive=false;invSetPhase('done','Investigation complete','Download the JSON now and upload it here. It contains waypoint scans, the full travel stream, sensor arrays, timestamps and route labels.');
        $('invStatus').textContent='Complete · '+investigationSamples+' samples · '+invLocationCount+' locations';
      } else {
        $('invNext').textContent='Next · begin travel';invSetPhase('capture_done','Scan complete: '+label,'The capture timer has finished. Review the rover position. Press Next to start logging the journey to the next location, or Last only when this is the final destination.');
        $('invStatus').textContent='Scan complete · '+label+' · '+investigationSamples+' samples';
      }}
    invUi();};tick();invTimer=setInterval(tick,100);
}
$('invStart').onclick=()=>{
  if(!latestSnapshot||!latestSnapshot.sensors){$('invStatus').textContent='Waiting for live sensor data';return;}
  if(autoOn){$('invStatus').textContent='Disable autonomy first. Investigation uses supervised/manual movement.';return;}
  investigationEvents=[];investigationSamples=0;invLocationCount=0;invLastEventMs=0;investigationStartIso=new Date().toISOString();investigationActive=true;
  invEvent('INVESTIGATION_START','START',{robot:{box_id:4,rover_id:'R04',wheel_encoders:false,lidar_bins:90,lidar_bin_angle_deg:4},firmware_mode:'investigation_logger_only',sample_rate_target_hz:4});
  invBeginCapture('START','START_SCAN');$('invStatus').textContent='Started · recording sensor stream';invUi();
};
$('invNext').onclick=()=>{
  if(!investigationActive)return;
  const label=$('invLabel').value.trim()||('LOCATION_'+(invLocationCount+1));
  if(invPhase==='capture_done'){
    invEvent('TRAVEL_START',label,{operator_instruction:'Move the rover manually toward the next location. Continuous sensor frames are recorded while travelling; no encoder distance is available.'});
    invSetPhase('travel','Travelling to: '+label,'Continuous route recording is active. Drive manually using the pad. At the next location, update the label and press the Arrived button. If this is the final destination, press Last instead.');
    $('invNext').textContent='Arrived · capture this location';$('invStatus').textContent='Travel logging · '+label;
  } else if(invPhase==='travel'){
    invEvent('TRAVEL_END',label,{operator_instruction:'Operator marks arrival at this location; no automatic arrival detection.'});
    invBeginCapture(label,'WAYPOINT_SCAN');$('invNext').textContent='Next · begin travel';
  }
};
$('invLast').onclick=()=>{
  if(!investigationActive||invPhase!=='travel')return;
  const label=$('invLabel').value.trim()||'END';
  invEvent('TRAVEL_END',label,{operator_instruction:'Operator confirms arrival; no automatic location/arrival detection.'});
  invBeginCapture(label,'END_SCAN');
};
// Every dashboard poll is preserved while active. During travel, this is the route trajectory evidence;
// during scans it captures the changing LiDAR and IMU stream at the dashboard's ~4 Hz update rate.
function invOnSnapshot(j){if(!investigationActive||!j.sensors)return;invSample(invPhase==='travel'?'TRAVEL_SAMPLE':(invPhase==='capturing'||invPhase==='final_capturing'?'SCAN_SAMPLE':'ROUTE_SAMPLE'));invUi();}
$('invDownload').onclick=()=>{
  if(!investigationEvents.length)return;
  const out={schema_version:'CAS-B3-R04-investigation-2.0',created_at:new Date().toISOString(),started_at:investigationStartIso,completed_at:new Date().toISOString(),
    robot:{box_id:4,rover_id:'R04',wheel_encoders:false,lidar_bins:90,lidar_bin_angle_deg:4},
    purpose:'Offline analysis dataset for indoor route/localization investigation; not a validated metric map or autonomous route.',
    capture:{dashboard_poll_target_hz:4,waypoint_scan_duration_ms:invCaptureDurationMs,route_samples_continuous:true},
    notes:$('operatorNotes').value.trim()||null,event_count:investigationEvents.length,sample_count:investigationSamples,location_count:invLocationCount,events:investigationEvents,
    caveats:['Sensor snapshots are browser-received frames, not hardware-synchronized samples.','No wheel encoders: motion distance cannot be derived reliably from commands or elapsed time.','LiDAR scan matching may estimate relative motion only after a scan-matching/localization algorithm is implemented and validated.','LiDAR bin orientation must be physically verified.','Landmark labels and arrival events are operator supplied; no automatic landmark recognition is implemented.','The logger does not autonomously rotate the rover; use slow supervised rotation only when safe.']};
  const blob=new Blob([JSON.stringify(out)],{type:'application/json;charset=utf-8'}),url=URL.createObjectURL(blob),a=document.createElement('a');
  a.href=url;a.download='CAS_B3_R04_investigation_'+new Date().toISOString().replace(/[:.]/g,'-')+'.json';document.body.appendChild(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(url),1000);
};

// Download a single self-contained JSON diagnostic: exact dashboard snapshot, raw sensor arrays,
// UI/autonomy state, all named values, timestamps and operator observations.
$('diagDownload').onclick=()=>{
  if(!latestSnapshot){$('diagIssue').textContent='No live snapshot yet; wait for the dashboard to come online.';return;}
  const values={};(latestSnapshot.values||[]).forEach(v=>{values[v.name]=v.value;});
  const out={schema_version:'CAS-B3-R04-diagnostic-1.0',exported_at:new Date().toISOString(),
    source:'CAS Box dashboard diagnostic export',robot:{box_id:4,rover_id:'R04',wheel_encoders:false},
    browser:{user_agent:navigator.userAgent,online:navigator.onLine,autonomy_enabled:!!autoOn,
      snapshot_received_at:new Date(latestSnapshot.receivedAt||Date.now()).toISOString(),
      snapshot_age_ms:Math.max(0,Date.now()-(latestSnapshot.receivedAt||Date.now())),
      recording:{active:logging,sample_count:logSampleCount}},
    dashboard:{state:latestSnapshot.state||null,values:values,cycle:latestSnapshot.cycle||null,
      sensors:latestSnapshot.sensors||null},
    operator_notes:$('operatorNotes').value.trim()||null,
    interpretation_notes:['LiDAR value 0 means no valid return in that bin.','Ultrasonic 300 cm may mean no echo/out of range, not confirmed free space.',
      'Snapshot values are one instant; compare exported_at and sensor ages before interpreting.',
      'No wheel encoders: commanded motion does not prove physical motion.']};
  const blob=new Blob([JSON.stringify(out,null,2)],{type:'application/json;charset=utf-8'});
  const url=URL.createObjectURL(blob),a=document.createElement('a');
  a.href=url;a.download='CAS_B3_R04_diagnostic_'+new Date().toISOString().replace(/[:.]/g,'-')+'.json';
  document.body.appendChild(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(url),1000);
};

function renderDiagnostics(j){
  const vals={};(j.values||[]).forEach(v=>vals[v.name]=v.value);
  const connected=vals['rover.ip'] && vals['rover.ip']!=='0.0.0.0';
  $('diagLink').textContent=connected?('Connected · '+vals['rover.ip']):'NOT CONNECTED';
  $('diagCmd').textContent=(vals['nav.command_l']||'0.00')+' / '+(vals['nav.command_r']||'0.00');
  $('diagSpeed').textContent=vals['nav.speed']?Number(vals['nav.speed']).toFixed(2):'—';
  $('diagRecoveries').textContent=vals['nav.recovery']||'0';
  const S=j.sensors||{},L=S.lidar||{},us=S.us||[];
  $('diagLidar').textContent=L.ok?('OK · '+L.pps+' pkt/s · CRC '+L.crc+'%'):'NO DATA';
  $('diagFront').textContent=(us[0]===undefined?'—':us[0]+' cm');
  const state=(j.state||'').toLowerCase(),diag=vals['nav.diagnostic']||'';
  let issue='No obvious sensor/link fault detected from current snapshot.';
  let advice='Keep autonomy off until you have confirmed manual drive works and the rover is on a clear, level floor.';
  let good=false;
  if(!connected){issue='ROVER LINK MISSING: the box has not found the rover.';advice='Power the box first, then reboot/power the rover so it joins CAS-Box-04. Check rover battery and the rover station on the box Wi-Fi.';}
  else if(!L.ok || Number(L.crc)>5){issue='LIDAR FAULT: scan data is unavailable or checksum errors are high.';advice='Disable autonomy. Check LiDAR power, UART wiring and the live scan display.';}
  else if(state.includes('fault') || state.includes('stop:') || state.includes('recovery')){issue=j.state+'. '+diag;advice='Read the original fault and diagnostic above. Do not keep re-enabling autonomy to force motion; inspect the reported clearance, sensor freshness, motor polarity and power first.';}
  else if(!j.sensors){issue='No sensor snapshot received.';advice='Wait for the sensor snapshot; check serial output and sensor initialization.';}
  else {good=true; issue='LIVE: '+(j.state||'State unavailable')+'. '+diag; advice='If commanded speeds are nonzero but the rover remains physically still, this dashboard cannot prove wheel motion without encoders. Check motor battery, motor/driver and wheel binding.';}
  $('diagIssue').textContent=issue;$('diagIssue').className='issue'+(good?' okIssue':'');$('diagAdvice').textContent=advice;
}
// ---- poll the box 4 times per second; log the same snapshot displayed by the dashboard
async function poll(){
  try{
    const j=await (await fetch('/values',{cache:'no-store'})).json();
    $('conn').textContent='Online';
    if(autoOn)fetch('/autonomy?hb=1',{cache:'no-store'}).catch(()=>{});
    $('state').textContent=j.state||'-';
    if(j.safety){
      const locked=!!j.safety.drive_inhibited;
      $('safetyState').textContent=locked?'STOP LATCHED · movement blocked':'Drive interlock clear';
      $('clearStop').disabled=!locked;
    }
    latestSnapshot={...j,receivedAt:Date.now()};
    // Refresh guided investigation button availability whenever a live snapshot arrives.
    // Previously invUi() ran only once at page load (before latestSnapshot existed), so
    // invStart remained disabled forever even though live sensor data was updating.
    invUi();
    renderSensors(j.sensors);renderDiagnostics(j);
    if(investigationActive && j.sensors){ invOnSnapshot(j); $('invStatus').textContent=(invPhase==='travel'?'Travel logging':(invPhase.indexOf('captur')===0?'Capture logging':'Recording'))+' · '+investigationSamples+' samples'; }
    if(logging&&j.sensors){
      logRows.push(rowFor('SAMPLE','',j.sensors).map(csvCell).join(','));
      logSampleCount++;updateLogUi();
    }
    if(dev){
      $('valTable').innerHTML=(j.values||[]).map(v=>`<tr><td class='k'>${esc(v.name)}</td><td>${esc(v.value)}</td></tr>`).join('');
      const c=j.cycle||{};$('cycle').textContent=`${c.min} / ${c.avg} / ${c.max} ms`;
    }
  }catch(e){$('conn').textContent='Offline';invUi();}
}
invUi();setInterval(poll,250);poll();
</script></body></html>)rawpage";

// Appends s to out as JSON string content (quotes and backslashes escaped, control chars dropped).
static void jsonEscape(String& out, const char* s) {
  for (; *s; ++s) {
    const char c = *s;
    if (c == '"' || c == '\\') { out += '\\'; out += c; }
    else if ((uint8_t)c >= 0x20) out += c;
  }
}

// ---------------------------------------------------------------------------------------------

// Takes the mutex; creates it on first use (always in setup(), before the server task runs).
void WebHMI::_lock() {
  if (!_mtx) _mtx = xSemaphoreCreateMutex();
  xSemaphoreTake(_mtx, portMAX_DELAY);
}

void WebHMI::begin(const char* ssid, const char* pass, IPAddress ip, uint8_t channel) {
  _lock();  // make sure the mutex exists before the server starts
  xSemaphoreGive(_mtx);

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);                                  // no Wi-Fi power save on the box
  WiFi.softAPConfig(ip, ip, IPAddress(255, 255, 255, 0));
  if (WiFi.softAP(ssid, pass, channel)) {
    Serial.printf("[WebHMI] AP %s, channel %d, IP %s\n", ssid, WiFi.channel(),
                  WiFi.softAPIP().toString().c_str());
  } else {
    Serial.println("[WebHMI] WiFi.softAP failed");
  }

  _server = new AsyncWebServer(80);
  _server->on("/", HTTP_GET, [this](AsyncWebServerRequest* req) {
    req->send(200, "text/html; charset=utf-8", _buildPage());
  });
  _server->on("/values", HTTP_GET, [this](AsyncWebServerRequest* req) {
    req->send(200, "application/json", _buildJson());
  });
  _server->on("/clock", HTTP_GET, [](AsyncWebServerRequest* req) {
    const uint32_t deviceMs = millis();
    req->send(200, "application/json", String("{\"device_ms\":") + String(deviceMs) + "}");
  });
  _server->on("/camera/register", HTTP_GET, [this](AsyncWebServerRequest* req) {
    if (!req->hasParam("ip")) { req->send(400, "application/json", "{\"ok\":false}"); return; }
    _cameraIp = req->getParam("ip")->value(); _cameraLastSeenMs = millis();
    req->send(200, "application/json", "{\"ok\":true}");
  });
  _server->on("/camera/status", HTTP_GET, [this](AsyncWebServerRequest* req) {
    const bool online = !_cameraIp.isEmpty() && millis() - _cameraLastSeenMs < 15000;
    String out = "{\"online\":" + String(online ? "true" : "false") + ",\"ip\":\"" + _cameraIp +
      "\",\"age_ms\":" + String(_cameraIp.isEmpty() ? 0 : millis() - _cameraLastSeenMs) + "}";
    req->send(200, "application/json", out);
  });
  _server->on("/camera/snapshot", HTTP_GET, [this](AsyncWebServerRequest* req) {
    if (_cameraIp.isEmpty() || millis() - _cameraLastSeenMs >= 15000) {
      req->send(503, "application/json", "{\"ok\":false,\"reason\":\"camera_offline\"}"); return;
    }
    HTTPClient http; http.setTimeout(1800);
    if (!http.begin(String("http://") + _cameraIp + "/snapshot")) {
      req->send(502, "application/json", "{\"ok\":false,\"reason\":\"camera_connect_failed\"}"); return;
    }
    const int code = http.GET(); const int length = http.getSize();
    if (code != 200 || length <= 0 || length > 64 * 1024) {
      http.end(); req->send(502, "application/json", "{\"ok\":false,\"reason\":\"invalid_camera_frame\"}"); return;
    }
    String jpeg = http.getString(); http.end();
    if (jpeg.length() < 4 || (uint8_t)jpeg[0] != 0xff || (uint8_t)jpeg[1] != 0xd8 ||
        (uint8_t)jpeg[jpeg.length()-2] != 0xff || (uint8_t)jpeg[jpeg.length()-1] != 0xd9) {
      req->send(502, "application/json", "{\"ok\":false,\"reason\":\"invalid_jpeg\"}"); return;
    }
    AsyncWebServerResponse* response = req->beginResponse(200, "image/jpeg", jpeg);
    response->addHeader("Cache-Control", "no-store"); response->addHeader("X-Image-Source", "camera-relay");
    req->send(response);
  });
  _server->on("/navigation/instructions", HTTP_GET, [this](AsyncWebServerRequest* req) {
    if (!req->hasParam("json")) { req->send(400, "application/json", "{\"ok\":false,\"reason\":\"missing_json\"}"); return; }
    const String body = req->getParam("json")->value();
    if (body.length() > 2048 || body.indexOf("\"mode\":\"navigation\"") < 0 || body.indexOf("\"waypoints\"") < 0) {
      req->send(400, "application/json", "{\"ok\":false,\"reason\":\"invalid_compact_navigation_package\"}"); return;
    }
    // Intake is deliberately non-actuating. The live sensor/safety controller must validate
    // and explicitly enable any future execution path.
    req->send(200, "application/json", "{\"ok\":true,\"accepted\":true,\"actuating\":false}");
  });
  _server->on("/autonomy", HTTP_GET, [this](AsyncWebServerRequest* req) {
    if (req->hasParam("on")) {
      const bool on = req->getParam("on")->value() == "1";
      if (on && driveInhibited()) {
        req->send(423, "application/json", "{\"autonomy\":false,\"reason\":\"safety stop latched\"}");
        return;
      }
      portENTER_CRITICAL(&_driveMux);
      _autonomyEnabled = on;
      _autonomyHeartbeatMs = on ? millis() : 0;
      _driveL = _driveR = 0;
      _driveMs = millis();
      portEXIT_CRITICAL(&_driveMux);
      req->send(200, "application/json", on ? "{\"autonomy\":true}" : "{\"autonomy\":false}");
      return;
    }
    if (req->hasParam("hb")) {
      portENTER_CRITICAL(&_driveMux);
      if (_autonomyEnabled) _autonomyHeartbeatMs = millis();
      portEXIT_CRITICAL(&_driveMux);
      req->send(200, "application/json", "{\"ok\":true}");
      return;
    }
    req->send(400, "application/json", "{\"ok\":false}");
  });
  _server->on("/safety/stop", HTTP_POST, [this](AsyncWebServerRequest* req) {
    stopDrive();
    req->send(200, "application/json", "{\"ok\":true,\"drive_inhibited\":true}");
  });
  _server->on("/safety/clear", HTTP_POST, [this](AsyncWebServerRequest* req) {
    clearDriveInhibit();
    req->send(200, "application/json", "{\"ok\":true,\"drive_inhibited\":false}");
  });
  _server->on("/safety/state", HTTP_GET, [this](AsyncWebServerRequest* req) {
    req->send(200, "application/json", driveInhibited() ? "{\"drive_inhibited\":true}" : "{\"drive_inhibited\":false}");
  });
  _server->on("/drive", HTTP_GET, [this](AsyncWebServerRequest* req) {
    if (!req->hasParam("l") || !req->hasParam("r") || !req->hasParam("id") || !req->hasParam("n")) {
      req->send(400, "application/json", "{\"ok\":false}");
      return;
    }
    const float l = constrain(req->getParam("l")->value().toFloat(), -_driveMax, _driveMax);
    const float r = constrain(req->getParam("r")->value().toFloat(), -_driveMax, _driveMax);
    const uint32_t id = strtoul(req->getParam("id")->value().c_str(), nullptr, 10);
    const uint32_t n = strtoul(req->getParam("n")->value().c_str(), nullptr, 10);
    bool fresh = false, inhibited = false;
    portENTER_CRITICAL(&_driveMux);
    inhibited = _driveInhibited || _autonomyEnabled;
    // a new page (id) starts over; within a page only a higher number counts
    fresh = !inhibited && ((id != _drivePageId) || (n > _driveSeq));
    if (fresh) {
      _drivePageId = id;
      _driveSeq = n;
      _driveL = l;
      _driveR = r;
      _driveMs = millis();
    }
    portEXIT_CRITICAL(&_driveMux);
    if (inhibited) req->send(423, "application/json", "{\"ok\":false,\"reason\":\"drive inhibited or autonomy active\"}");
    else req->send(200, "application/json", fresh ? "{\"ok\":true}" : "{\"ok\":false,\"stale\":true}");
  });
  _server->begin();
  _cycleLastUs = micros();
  _cycleWinMs = millis();
}

void WebHMI::setTitle(const String& title) { _title = title; }

void WebHMI::enableDrivePad(float maxSpeed) {
  _drivePad = true;
  _driveMax = maxSpeed;
}

void WebHMI::setStateText(const String& text) {
  _lock();
  _state = text;
  xSemaphoreGive(_mtx);
}

void WebHMI::setSensorJson(const String& json) {
  _lock();
  _sensorJson = json;
  xSemaphoreGive(_mtx);
}

void WebHMI::showValue(const char* name, double value) { _setValue(name, String(value, 3)); }
void WebHMI::showValue(const char* name, const String& value) { _setValue(name, value); }

// Updates the value with this name or takes the first free slot (ignored if all are used).
void WebHMI::_setValue(const char* name, const String& text) {
  _lock();
  Value* slot = nullptr;
  for (Value& v : _values) {
    if (v.inUse && strncmp(v.name, name, sizeof(v.name)) == 0) { slot = &v; break; }
    if (!v.inUse && !slot) slot = &v;
  }
  if (slot) {
    if (!slot->inUse) {
      strlcpy(slot->name, name, sizeof(slot->name));
      slot->inUse = true;
    }
    strlcpy(slot->text, text.c_str(), sizeof(slot->text));
  }
  xSemaphoreGive(_mtx);
}

void WebHMI::getDrive(float& left, float& right, uint32_t& ageMs) const {
  portENTER_CRITICAL(&_driveMux);
  left = _driveL;
  right = _driveR;
  const uint32_t ms = _driveMs;
  portEXIT_CRITICAL(&_driveMux);
  ageMs = ms ? millis() - ms : UINT32_MAX;  // millis() read after ms: never negative
}

bool WebHMI::autonomyEnabled() const {
  portENTER_CRITICAL(&_driveMux);
  const bool enabled = _autonomyEnabled;
  portEXIT_CRITICAL(&_driveMux);
  return enabled;
}

bool WebHMI::driveInhibited() const {
  portENTER_CRITICAL(&_driveMux);
  const bool inhibited = _driveInhibited;
  portEXIT_CRITICAL(&_driveMux);
  return inhibited;
}

void WebHMI::stopDrive() {
  portENTER_CRITICAL(&_driveMux);
  _driveInhibited = true;
  _autonomyEnabled = false;
  _autonomyHeartbeatMs = 0;
  _driveL = _driveR = 0;
  _driveMs = millis();
  portEXIT_CRITICAL(&_driveMux);
}

void WebHMI::clearDriveInhibit() {
  portENTER_CRITICAL(&_driveMux);
  _driveL = _driveR = 0;
  _driveMs = millis();
  _driveInhibited = false;
  portEXIT_CRITICAL(&_driveMux);
}

bool WebHMI::autonomyHeartbeatFresh(uint32_t maxAgeMs) const {
  portENTER_CRITICAL(&_driveMux);
  const bool enabled = _autonomyEnabled;
  const uint32_t hb = _autonomyHeartbeatMs;
  portEXIT_CRITICAL(&_driveMux);
  return enabled && hb != 0 && (millis() - hb <= maxAgeMs);
}

// Loop time over 1 s windows: min / average / max time between two tick() calls.
void WebHMI::tick() {
  const uint32_t nowUs = micros();
  const uint32_t dt = nowUs - _cycleLastUs;
  _cycleLastUs = nowUs;
  if (_cycleCount == 0 || dt < _cycleMinUs) _cycleMinUs = dt;
  if (dt > _cycleMaxUs) _cycleMaxUs = dt;
  _cycleSumUs += dt;
  _cycleCount++;

  const uint32_t nowMs = millis();
  if (nowMs - _cycleWinMs < 1000) return;
  _cycleWinMs = nowMs;
  _lock();
  _cycleMinMs = _cycleMinUs / 1000.0f;
  _cycleMaxMs = _cycleMaxUs / 1000.0f;
  _cycleAvgMs = _cycleSumUs / 1000.0f / _cycleCount;
  xSemaphoreGive(_mtx);
  _cycleCount = 0;
  _cycleSumUs = 0;
  _cycleMaxUs = 0;
}

static const char kCompactPage[] PROGMEM = R"compact(<!doctype html><html><head><meta charset='utf-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'><title>%TITLE%</title>
<style>
:root{font-family:system-ui;background:#10151c;color:#eef3f8}*{box-sizing:border-box}body{margin:0}.wrap{max-width:760px;margin:auto;padding:14px}.top{display:flex;justify-content:space-between;align-items:center}.title{font-size:20px;font-weight:700}.state{color:#aebdcc;font-size:13px}.card{background:#19222d;border:1px solid #2b3948;border-radius:12px;padding:13px;margin-top:12px}.label{color:#aebdcc;font-size:12px}.row{display:flex;gap:8px;align-items:center;flex-wrap:wrap}button,input,select{font:inherit;border-radius:8px;border:1px solid #405264;background:#243241;color:#eef3f8;padding:10px}button{cursor:pointer;font-weight:650}.primary{background:#2878c8;border-color:#4396ee}.danger{background:#b7353e;border-color:#e65e66}.good{color:#6ce0a2}.warn{color:#ffd166}.grid{display:grid;grid-template-columns:repeat(2,1fr);gap:8px}.tile{background:#111820;padding:9px;border-radius:8px}.tile b{display:block;margin-top:3px;font-size:16px}.drive{display:grid;grid-template-columns:repeat(3,70px);gap:8px;justify-content:center;margin:12px 0}.drive button{height:58px;font-size:22px}.wide{width:100%}#log{white-space:pre-wrap;font-size:12px;color:#aebdcc;max-height:110px;overflow:auto}
</style></head><body><main class='wrap'><div class='top'><div><div class='title'>%TITLE%</div><div class='state' id='state'>Connecting…</div></div><button class='danger' id='stop'>STOP ROVER</button></div>
<section class='card'><div class='label'>ROUTE MAPPING SESSION</div><div class='row' style='margin-top:8px'><input id='route' class='wide' maxlength='48' placeholder='Route name, e.g. Block-B-to-Block-H'></div><div class='row' style='margin-top:8px'><button id='start' class='primary'>Start route log</button><button id='finish'>Finish & download log</button><span id='logState' class='label'>Not logging</span></div></section>
<section class='card'><div class='label'>LIVE SENSOR SUMMARY</div><div class='grid' style='margin-top:8px'><div class='tile'>LiDAR <b id='lidar'>—</b></div><div class='tile'>Ultrasonic front <b id='front'>—</b></div><div class='tile'>IMU yaw <b id='yaw'>—</b></div><div class='tile'>GPS <b id='gps'>—</b></div><div class='tile'>Rover link <b id='rover'>—</b></div><div class='tile'>Camera <b id='camera'>—</b></div></div></section>
<section class='card'><div class='label'>MANUAL ROUTE DRIVE</div><div class='drive'><span></span><button data-v='f'>▲</button><span></span><button data-v='l'>◀</button><button class='danger' data-v='s'>■</button><button data-v='r'>▶</button><span></span><button data-v='b'>▼</button><span></span></div><div class='row'><label class='label'>Speed <input id='speed' type='range' min='.05' max='%MAXSPD%' step='.05' value='.15'></label><span id='speedValue'>0.15</span></div></section>
<section class='card'><div class='label'>IMAGE CAPTURE</div><div class='row' style='margin-top:8px'><button id='capture' class='primary'>Capture image + sensor record</button><span id='captureState' class='label'>Images download to this phone.</span></div></section>
<section class='card'><div class='label'>SESSION EVENTS</div><div id='log'>No route log yet.</div></section></main>
<script>
const $=id=>document.getElementById(id),MAX=%MAXSPD%;let samples=[],logging=false,seq=0,timer=null,last=null,db;
const ready=new Promise((ok,no)=>{const r=indexedDB.open('CAS_B3_R04_INVESTIGATION',1);r.onupgradeneeded=()=>r.result.createObjectStore('items',{autoIncrement:true});r.onsuccess=()=>{db=r.result;ok()};r.onerror=()=>no(r.error)});
function put(v){return ready.then(()=>new Promise((ok,no)=>{const t=db.transaction('items','readwrite');t.objectStore('items').add(v);t.oncomplete=ok;t.onerror=()=>no(t.error)}))}
function all(){return ready.then(()=>new Promise((ok,no)=>{const t=db.transaction('items');const r=t.objectStore('items').getAll();r.onsuccess=()=>ok(r.result);r.onerror=()=>no(r.error)}))}
function download(name,data,type){const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([data],{type}));a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000)}
function event(type,data={}){const e={investigation_id:$('route').value||'unnamed',sequence:++seq,timestamp:new Date().toISOString(),type,...data};samples.push(e);put({kind:'event',value:e});$('log').textContent=samples.slice(-8).map(x=>x.sequence+'  '+x.type+'  '+x.timestamp).join('\n')}
function drive(l,r){fetch('/drive?l='+l.toFixed(2)+'&r='+r.toFixed(2)+'&id=compact&n='+(++seq)).catch(()=>{})}
function stop(){clearInterval(timer);timer=null;drive(0,0)}
$('stop').onclick=()=>{stop();fetch('/safety/stop',{method:'POST'});$('state').textContent='STOP LATCHED'};
$('start').onclick=async()=>{if(logging)return;await ready;logging=true;samples=[];seq=0;event('INVESTIGATION_START');$('logState').textContent='Recording continuously on phone';$('start').disabled=true};
function crc32(a){let c=~0;for(const b of a){c^=b;for(let i=0;i<8;i++)c=(c>>>1)^((c&1)?0xedb88320:0)}return ~c>>>0}
function u16(n){return new Uint8Array([n&255,n>>>8&255])}function u32(n){return new Uint8Array([n&255,n>>>8&255,n>>>16&255,n>>>24&255])}
async function makeZip(entries){let out=[],central=[],offset=0;const enc=new TextEncoder();for(const e of entries){const n=enc.encode(e.name),d=typeof e.data==='string'?enc.encode(e.data):new Uint8Array(await e.data.arrayBuffer()),c=crc32(d),h=new Uint8Array([...new Uint8Array([80,75,3,4]),...u16(20),...u16(0),...u16(0),...u16(0),...u16(0),...u32(c),...u32(d.length),...u32(d.length),...u16(n.length),...u16(0),...n]);out.push(h,d);central.push({n,d,c,offset});offset+=h.length+d.length}const start=offset;for(const x of central){const h=new Uint8Array([...new Uint8Array([80,75,1,2]),...u16(20),...u16(20),...u16(0),...u16(0),...u16(0),...u16(0),...u32(x.c),...u32(x.d.length),...u32(x.d.length),...u16(x.n.length),...u16(0),...u16(0),...u16(0),...u16(0),...u32(0),...u32(x.offset),...x.n]);out.push(h);offset+=h.length}const size=offset-start;out.push(new Uint8Array([...new Uint8Array([80,75,5,6]),...u16(0),...u16(0),...u16(central.length),...u16(central.length),...u32(size),...u32(start),...u16(0)]));return new Blob(out,{type:'application/zip'})}
$('finish').onclick=async()=>{if(!logging)return;stop();event('ROUTE_FINISH');event('INVESTIGATION_FINALIZATION_STARTED');$('logState').textContent='Finalizing phone dataset…';const items=await all(),events=items.filter(x=>x.kind==='event').map(x=>x.value),telemetry=items.filter(x=>x.kind==='telemetry').map(x=>x.value),images=items.filter(x=>x.kind==='image');const frameRows=['frame_id,filename,capture_timestamp,bytes,sensor_record_sequence'];const entries=[{name:'README.txt',data:'CAS B3/R04 phone investigation dataset\nOriginal JPEGs and sensor records captured by the phone.\n'}, {name:'events.jsonl',data:events.map(JSON.stringify).join('\n')+'\n'},{name:'telemetry.jsonl',data:telemetry.map(JSON.stringify).join('\n')+'\n'}];for(const im of images){entries.push({name:'camera/frames/'+im.value.filename,data:im.blob});frameRows.push([im.value.frame_id,im.value.filename,im.value.timestamp,im.blob.size,im.value.sensor_sequence].map(x=>'"'+String(x??'').replaceAll('"','""')+'"').join(','))}entries.push({name:'camera/frames.csv',data:frameRows.join('\n')+'\n'},{name:'manifest.json',data:JSON.stringify({schema_version:'CAS-B3-R04-phone-investigation-1.0',investigation_id:$('route').value||'unnamed',finalized_at:new Date().toISOString(),event_count:events.length,telemetry_count:telemetry.length,image_count:images.length,storage:'phone IndexedDB',timestamp_alignment:'phone wall-clock capture time; ESP32 device timestamps retained in telemetry'},null,2)});const zip=await makeZip(entries);download('CAS_B3_R04_'+($('route').value||'investigation')+'.zip',zip,'application/zip');logging=false;$('start').disabled=false;$('logState').textContent='ZIP generated and downloaded to phone'};
$('capture').onclick=async()=>{if(!logging){$('captureState').textContent='Start Investigation first';return}const stamp=new Date().toISOString(),sensor=last;event('IMAGE_CAPTURE_REQUESTED',{sensor});try{const r=await fetch('/camera/snapshot?ts='+Date.now()),blob=await r.blob();if(!r.ok||blob.type.indexOf('image')<0)throw Error('camera capture failed');const id='frame_'+String(seq).padStart(6,'0');const filename=id+'.jpg';await put({kind:'image',blob,value:{frame_id:id,filename,timestamp:stamp,sensor_sequence:seq,sensor_snapshot:sensor}});event('IMAGE_SAVED',{frame_id:id,filename,timestamp:stamp});$('captureState').textContent=filename+' saved on phone'}catch(e){event('IMAGE_MISSING',{reason:e.message,timestamp:stamp});$('captureState').textContent='Capture failed; missing image recorded'}};
const dirs={f:[.15,.15],b:[-.15,-.15],l:[-.15,.15],r:[.15,-.15]};document.querySelectorAll('[data-v]').forEach(b=>{b.onpointerdown=e=>{e.preventDefault();if(b.dataset.v==='s'){stop();return}stop();const d=dirs[b.dataset.v],v=Number($('speed').value);drive(d[0]/.15*v,d[1]/.15*v);timer=setInterval(()=>drive(d[0]/.15*v,d[1]/.15*v),150)};['pointerup','pointercancel','lostpointercapture'].forEach(x=>b.addEventListener(x,stop))});$('speed').oninput=()=>{$('speedValue').textContent=Number($('speed').value).toFixed(2)};
async function poll(){try{const j=await (await fetch('/values',{cache:'no-store'})).json();last=j.sensors;$('state').textContent=j.state||'Connected';const s=j.sensors||{};$('lidar').textContent=s.lidar?.ok?'OK':'Unavailable';$('front').textContent=s.usFiltered?.[0]??'—';$('yaw').textContent=s.imu?.yaw!=null?Number(s.imu.yaw).toFixed(1)+'°':'—';$('gps').textContent=s.gps?.fix?'Fix':'No fix';$('rover').textContent=(j.values||[]).some(x=>x.name==='rover.ok')?'Connected':'Checking';const c=await (await fetch('/camera/status')).json();$('camera').textContent=c.online?'Ready':'Offline';if(logging&&last){const rec={timestamp:new Date().toISOString(),device:j,sequence:seq};await put({kind:'telemetry',value:rec});$('logState').textContent='Recording · phone storage'}}catch(e){$('state').textContent='Box connection lost'}}setInterval(poll,500);poll();
</script></body></html>)compact";

String WebHMI::_buildPage() const {
  String page = FPSTR(kCompactPage);
  page.replace("%TITLE%", _title);
  page.replace("%MAXSPD%", _drivePad ? String(_driveMax, 2) : String("0"));
  return page;
}

// {"state":"..","values":[{"name":..,"value":..}],"cycle":{min,avg,max},"sensors":{..}|null}
String WebHMI::_buildJson() const {
  String out;
  out.reserve(6000);
  xSemaphoreTake(_mtx, portMAX_DELAY);
  out += "{\"state\":\"";
  jsonEscape(out, _state.c_str());
  portENTER_CRITICAL(&_driveMux);
  const bool driveLocked = _driveInhibited;
  portEXIT_CRITICAL(&_driveMux);
  out += "\",\"safety\":{\"drive_inhibited\":";
  out += driveLocked ? "true" : "false";
  out += "},\"values\":[";
  bool first = true;
  for (const Value& v : _values) {
    if (!v.inUse) continue;
    if (!first) out += ',';
    first = false;
    out += "{\"name\":\"";
    jsonEscape(out, v.name);
    out += "\",\"value\":\"";
    jsonEscape(out, v.text);
    out += "\"}";
  }
  out += "],\"cycle\":{\"min\":";
  out += String(_cycleMinMs, 2);
  out += ",\"avg\":";
  out += String(_cycleAvgMs, 2);
  out += ",\"max\":";
  out += String(_cycleMaxMs, 2);
  out += "},\"sensors\":";
  out += _sensorJson.length() ? _sensorJson : String("null");
  out += '}';
  xSemaphoreGive(_mtx);
  return out;
}
