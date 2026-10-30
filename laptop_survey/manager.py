"""Laptop-owned survey collection. Camera image traffic bypasses the main ESP32."""
from __future__ import annotations
import csv, hashlib, json, os, secrets, shutil, threading, time, uuid, zipfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
import requests
from analysis import analyse_lidar

SCHEMA = "CAS-B3-R04-survey-1.0"
VERSION = "0.1.0"
POLL_S = 0.22
MAX_DRIVE = 0.25
US_NAMES = ["front", "left", "right", "back", "front_angled"]
TAGS = {
 "tactile_guidance_visible","tactile_guidance_partial","tactile_guidance_missing",
 "tactile_guidance_junction","open_paved_area","possible_wall_left","possible_wall_right",
 "narrow_passage","pillar_or_column","obstacle","cobblestone_rough_paving",
 "tile_surface_transition","ramp","steps","drainage_feature","pedestrian_present",
 "store_or_landmark","unknown_needs_review","wet_surface","raised_threshold",
 "tactile_warning_dots","covered_colonnade"
}
def now_iso():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00","Z")
def write_jsonl(path: Path, obj: dict):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(obj, separators=(",",":"), ensure_ascii=False, allow_nan=False)+"\n")
        f.flush()
def write_csv(path: Path, header: list, row: list):
    path.parent.mkdir(parents=True, exist_ok=True)
    is_new = not path.exists() or path.stat().st_size == 0
    with path.open("a", encoding="utf-8", newline="") as f:
        w=csv.writer(f)
        if is_new: w.writerow(header)
        w.writerow(row); f.flush()
def values_dict(payload):
    return {str(v.get("name")):str(v.get("value","")) for v in payload.get("values",[]) if isinstance(v,dict)}

class SurveyManager:
    def __init__(self, box_url: str, data_root: Path, camera_token: str):
        self.box_url=box_url.rstrip("/")
        self.data_root=data_root.resolve()
        self.sessions_root=self.data_root/"sessions"
        self.archives_root=self.data_root/"archives"
        self.sessions_root.mkdir(parents=True,exist_ok=True)
        self.archives_root.mkdir(parents=True,exist_ok=True)
        self.camera_token=camera_token
        self.lock=threading.RLock()
        self.stop_event=threading.Event()
        self.worker=None
        self.latest=None
        self.box_online=False
        self.box_last_error=None
        self.box_last_receive_at=None
        self.box_last_receive_mono_ns=None
        self.poll_failures=0
        self.total_poll_failures=0
        self.total_missing_samples=0
        self.session=None
        self.drive_enabled=False
        self.estop_latched=True
        self.drive_page_id=secrets.randbelow(2_000_000_000)+1
        self.drive_sequence=0
        self.last_snapshot_seq=None
        self.clock_samples=[]
        self.pending_prompt=None
        self.open_area_suggestion=None
        self.last_wall_key=None
        self.last_wall_prompt_mono=0.0
        self.camera_frames_received=0
        self.camera_errors=0
        self.camera_missing_frames=0
        self.camera_last_frame_at=None
        self.camera_last_frame_mono=0.0
        self.camera_last_heartbeat_at=None
        self.camera_last_heartbeat_mono=0.0
        self.camera_board_status={}
        self.last_vision=None
        self.capture_requests=[]
        self.camera_accepting=True
        self.camera_uploads_inflight=0
        self.camera_upload_condition=threading.Condition(self.lock)
        self._mark_interrupted_sessions()

    def start_worker(self):
        if self.worker and self.worker.is_alive(): return
        self.stop_event.clear()
        self.worker=threading.Thread(target=self._poll_loop,name="cas-box-poller",daemon=True)
        self.worker.start()

    def shutdown(self):
        self.stop_event.set()
        if self.worker: self.worker.join(timeout=2)

    def _mark_interrupted_sessions(self):
        for folder in self.sessions_root.iterdir():
            p=folder/"session"/"metadata.json"
            if not folder.is_dir() or not p.exists(): continue
            try:
                meta=json.loads(p.read_text())
                if meta.get("state") in ("active","finalizing") and not (folder/"integrity"/"data_quality_report.json").exists():
                    meta["state"]="interrupted_by_laptop_service_restart"
                    meta["recovery_note"]="Raw records retained; server did not resume this session."
                    meta["interrupted_at"]=now_iso()
                    p.write_text(json.dumps(meta,indent=2))
                    write_jsonl(folder/"annotations"/"investigation_events.jsonl",{
                        "event_id":str(uuid.uuid4()),"type":"SERVICE_RESTART_INTERRUPTION","timestamp":now_iso()
                    })
            except Exception:
                pass

    def box_request(self, method, path, params=None, timeout=0.8):
        return requests.request(method,self.box_url+path,params=params,timeout=timeout)

    def read_values(self):
        r=self.box_request("GET","/values",timeout=1.2)
        r.raise_for_status()
        p=r.json()
        if not isinstance(p,dict) or not isinstance(p.get("sensors"),dict):
            raise RuntimeError("Box /values did not return a sensor object")
        return p

    def _poll_loop(self):
        next_clock=time.monotonic()+2
        while not self.stop_event.is_set():
            t0=time.monotonic()
            try:
                payload=self.read_values()
                wall=now_iso(); mono=time.monotonic_ns()
                with self.lock:
                    was=self.box_online
                    self.box_online=True; self.poll_failures=0; self.box_last_error=None
                    self.box_last_receive_at=wall; self.box_last_receive_mono_ns=mono
                    self.latest={**payload,"_laptop_received_at":wall,"_laptop_received_monotonic_ns":mono}
                    if not was: self._connection_event("BOX_RECONNECTED")
                    self._record_sample(payload,wall,mono)
                    self._analyse_navigation(payload)
            except Exception as exc:
                with self.lock:
                    was=self.box_online
                    self.box_online=False
                    self.poll_failures+=1; self.total_poll_failures+=1
                    self.box_last_error=f"{type(exc).__name__}: {exc}"
                    if was: self._connection_event("BOX_DISCONNECTED",self.box_last_error)
            if time.monotonic()>=next_clock:
                self._sync_clock()
                next_clock=time.monotonic()+10
            self.stop_event.wait(max(.02,POLL_S-(time.monotonic()-t0)))

    def _connection_event(self, typ, details=None):
        s=self.session
        if s and s["state"]=="active":
            write_jsonl(s["dir"]/"telemetry"/"connection_events.jsonl",{
                "event_id":str(uuid.uuid4()),"type":typ,"timestamp":now_iso(),"details":details
            })

    def _sync_clock(self):
        t0=time.monotonic_ns(); wall=now_iso()
        try:
            r=self.box_request("GET","/clock",timeout=.8); r.raise_for_status()
            device=int(r.json()["device_ms"]); t1=time.monotonic_ns()
            sample={"laptop_request_wall_time":wall,"laptop_monotonic_before_ns":t0,
                    "laptop_monotonic_after_ns":t1,"box_device_ms":device,
                    "rtt_ms":round((t1-t0)/1e6,3),
                    "estimated_laptop_monotonic_minus_box_ms":round(((t0+t1)/2/1e6)-device,3),
                    "box_clock_domain":"millis since main ESP32 boot; not UTC"}
            with self.lock:
                self.clock_samples.append(sample); self.clock_samples=self.clock_samples[-500:]
                if self.session and self.session["state"]=="active":
                    write_jsonl(self.session["dir"]/"telemetry"/"clock_sync.jsonl",sample)
        except Exception:
            pass

    def _record_sample(self,payload,wall,mono):
        s=self.session
        if not s or s["state"]!="active": return
        sensors=payload.get("sensors")
        if not isinstance(sensors,dict):
            s["quality"]["snapshots_without_sensors"]+=1; return
        seq=sensors.get("snapshot_seq")
        device_ms=sensors.get("snapshot_device_ms")
        if isinstance(seq,int) and self.last_snapshot_seq is not None:
            delta=seq-self.last_snapshot_seq
            if delta==0: return
            if delta>1:
                missing=delta-1
                s["quality"]["missing_box_snapshots"]+=missing
                self.total_missing_samples+=missing
                write_jsonl(s["dir"]/"telemetry"/"connection_events.jsonl",{
                    "event_id":str(uuid.uuid4()),"type":"BOX_SAMPLE_SEQUENCE_GAP","timestamp":wall,
                    "previous_seq":self.last_snapshot_seq,"current_seq":seq,"missing_estimate":missing})
            elif delta<0:
                write_jsonl(s["dir"]/"telemetry"/"connection_events.jsonl",{
                    "event_id":str(uuid.uuid4()),"type":"BOX_DEVICE_REBOOT_OR_SEQUENCE_RESET",
                    "timestamp":wall,"previous_seq":self.last_snapshot_seq,"current_seq":seq})
        if isinstance(seq,int): self.last_snapshot_seq=seq
        s["collector_seq"]+=1
        vals=values_dict(payload)
        rec={"schema_version":SCHEMA,"session_id":s["id"],"record_type":"sensor_snapshot",
             "collector_sequence":s["collector_seq"],"box_snapshot_sequence":seq,
             "box_snapshot_device_ms":device_ms,"box_response_device_ms":payload.get("box_response_device_ms"),
             "box_clock_domain":"millis since boot",
             "laptop_received_at":wall,"laptop_received_monotonic_ns":mono,
             "dashboard_state":payload.get("state"),"cycle":payload.get("cycle"),
             "sensors":sensors,"named_system_values":vals}
        try:
            write_jsonl(s["dir"]/"telemetry"/"sensor_snapshots.jsonl",rec)
            write_jsonl(s["dir"]/"telemetry"/"lidar_scans.jsonl",{
                "session_id":s["id"],"collector_sequence":s["collector_seq"],
                "box_snapshot_sequence":seq,"box_device_ms":device_ms,"box_response_device_ms":payload.get("box_response_device_ms"),
                "laptop_received_at":wall,"lidar":sensors.get("lidar")})
            self._imu_csv(s,rec); self._gps_csv(s,rec); self._us_csv(s,rec); self._system_csv(s,rec); self._motor_csv(s,rec)
            write_jsonl(s["dir"]/"telemetry"/"navigation_events.jsonl",{
                "timestamp":wall,"collector_sequence":s["collector_seq"],"device_ms":device_ms,
                "state":payload.get("state"),
                "navigation_values":{k:v for k,v in vals.items() if k.startswith(("nav.","drive."))}})
            s["quality"]["sensor_snapshots"]+=1
            s["quality"]["lidar_scans"]+=1
            self._quality(s,sensors)
            s["last_sample_at"]=wall; s["last_sample_device_ms"]=device_ms
        except OSError as exc:
            s["quality"]["storage_errors"]+=1
            s["quality"]["storage_error_last"]=f"{type(exc).__name__}: {exc}"
            s["state"]="storage_error"; s["error"]="Laptop disk write failure. Raw data is retained; stop and recover."
            self.drive_enabled=False; self.estop_latched=True
            threading.Thread(target=self._send_estop,daemon=True).start()

    def _imu_csv(self,s,r):
        i=r["sensors"].get("imu") or {}; raw=i.get("accel_m_s2") or [None]*3
        gyro=i.get("gyro_rad_s") or [None]*3; mag=i.get("mag_uT") or [None]*3
        write_csv(s["dir"]/"telemetry"/"imu.csv",
          ["session_id","collector_seq","device_ms","laptop_received_at","age_ms","valid","yaw_deg","pitch_deg","roll_deg",
           "cal_sys","cal_gyro","cal_accel","cal_mag","accel_x_m_s2","accel_y_m_s2","accel_z_m_s2",
           "gyro_x_rad_s","gyro_y_rad_s","gyro_z_rad_s","mag_x_uT","mag_y_uT","mag_z_uT","temperature_c"],
          [r["session_id"],r["collector_sequence"],r["box_snapshot_device_ms"],r["laptop_received_at"],i.get("age_ms"),
           i.get("ok"),i.get("yaw"),i.get("pitch"),i.get("roll"),*(i.get("cal") or [None]*4),*raw,*gyro,*mag,i.get("temperature_c")])

    def _gps_csv(self,s,r):
        g=r["sensors"].get("gps") or {}
        write_csv(s["dir"]/"telemetry"/"gps.csv",
          ["session_id","collector_seq","device_ms","laptop_received_at","location_age_ms","fix","satellites","hdop",
           "latitude_deg","longitude_deg","altitude_valid","altitude_m","speed_valid","speed_kmph",
           "course_valid","course_deg","utc_valid","gps_utc","chars_processed"],
          [r["session_id"],r["collector_sequence"],r["box_snapshot_device_ms"],r["laptop_received_at"],g.get("age_ms"),
           g.get("fix"),g.get("sats"),g.get("hdop"),g.get("lat") if g.get("fix") else None,
           g.get("lon") if g.get("fix") else None,g.get("altitude_valid"),g.get("altitude_m") if g.get("altitude_valid") else None,
           g.get("speed_valid"),g.get("speed_kmph") if g.get("speed_valid") else None,g.get("course_valid"),
           g.get("course_deg") if g.get("course_valid") else None,g.get("utc_valid"),g.get("utc") if g.get("utc_valid") else None,g.get("chars")])

    def _us_csv(self,s,r):
        x=r["sensors"]; arrays=[x.get(k) or [] for k in ("usRaw","usFiltered","usValid","usAgeMs","usTimeouts")]
        for i,name in enumerate(US_NAMES):
            vals=[a[i] if i<len(a) else None for a in arrays]
            write_csv(s["dir"]/"telemetry"/"ultrasonic.csv",
              ["session_id","collector_seq","device_ms","laptop_received_at","sensor_index","sensor_name",
               "raw_cm","median_cm","latest_valid","last_good_age_ms","timeout_count"],
              [r["session_id"],r["collector_sequence"],r["box_snapshot_device_ms"],r["laptop_received_at"],i,name,*vals])

    def _system_csv(self,s,r):
        v=r["named_system_values"]; cyc=r.get("cycle") or {}
        write_csv(s["dir"]/"telemetry"/"system_diagnostics.csv",
          ["session_id","collector_seq","device_ms","laptop_received_at","state","firmware","uptime_s","heap_kb",
           "min_heap_kb","ap_clients","rover_ip","rover_voltage_v","rover_rtt_ms","rover_ok","rover_fail","loop_min_ms","loop_avg_ms","loop_max_ms"],
          [r["session_id"],r["collector_sequence"],r["box_snapshot_device_ms"],r["laptop_received_at"],r.get("dashboard_state"),
           v.get("sys.firmware"),v.get("sys.uptime_s"),v.get("sys.heap_kB"),v.get("sys.min_heap_kB"),v.get("wifi.ap_clients"),
           v.get("rover.ip"),v.get("rover.voltage_V"),v.get("rover.rtt_ms"),v.get("rover.ok"),v.get("rover.fail"),
           cyc.get("min"),cyc.get("avg"),cyc.get("max")])

    def _motor_csv(self,s,r):
        v=r["named_system_values"]
        write_csv(s["dir"]/"telemetry"/"motor_commands.csv",
          ["session_id","timestamp","source","collector_seq","requested_left","requested_right",
           "box_reported_left","box_reported_right","command_age_ms","rover_voltage_v","rover_rtt_ms"],
          [r["session_id"],r["laptop_received_at"],"box_snapshot",r["collector_sequence"],None,None,
           v.get("drive.command_l"),v.get("drive.command_r"),v.get("drive.command_age_ms"),
           v.get("rover.voltage_V"),v.get("rover.rtt_ms")])

    def _quality(self,s,x):
        if not (x.get("imu") or {}).get("ok"): s["quality"]["imu_unavailable_samples"]+=1
        g=x.get("gps") or {}
        if not g.get("fix"): s["quality"]["gps_no_fix_samples"]+=1
        l=x.get("lidar") or {}
        if not l.get("ok"): s["quality"]["lidar_unhealthy_samples"]+=1
        age=l.get("age_ms")
        if not isinstance(age,(int,float)) or age<0 or age>1000: s["quality"]["lidar_stale_samples"]+=1
        for i,valid in enumerate(x.get("usValid") or []):
            if not valid:
                key=f"ultrasonic_{US_NAMES[i]}_invalid"
                s["quality"][key]=s["quality"].get(key,0)+1
        device=int(x.get("snapshot_device_ms") or 0)
        if device<s.get("last_device_ms",0): s["quality"]["device_clock_resets"]+=1
        s["last_device_ms"]=device

    def start_session(self,operator="",notes=""):
        with self.lock:
            if self.session and self.session.get("state") in {"active","finalizing","storage_error"}:
                raise RuntimeError("An active or recoverable session already exists")
        payload=self.read_values()
        values = values_dict(payload)
        if values.get("rover.connected", "false").lower() != "true":
            raise RuntimeError("R04 rover is not connected to CAS-Box-04 yet; wait for the RoverLink status to become connected.")
        if not self.camera_online():
            raise RuntimeError("AI-Thinker ESP32-CAM has not checked in yet. Wait for Camera online before starting the investigation.")
        stop=self.box_request("POST","/safety/stop",timeout=1.0)
        if stop.status_code!=200:
            raise RuntimeError(f"Survey firmware safety stop is required (HTTP {stop.status_code}); update the box firmware first")
        sid=datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")+"_"+uuid.uuid4().hex[:8]
        folder=self.sessions_root/sid
        for p in ("session","telemetry","camera/frames","annotations","derived","integrity"):
            (folder/p).mkdir(parents=True,exist_ok=True)
        session={"id":sid,"dir":folder,"state":"active","started_at":now_iso(),"operator":operator,"notes":notes,
                 "collector_seq":0,"last_sample_at":None,"camera_frame_count":0,"camera_sequence_last":None,
                 "camera_image_gaps":0,"poll_failures_at_start":self.total_poll_failures,
                 "quality":{"sensor_snapshots":0,"lidar_scans":0,"missing_box_snapshots":0,
                            "snapshots_without_sensors":0,"imu_unavailable_samples":0,"gps_no_fix_samples":0,
                            "lidar_unhealthy_samples":0,"lidar_stale_samples":0,"device_clock_resets":0,
                            "storage_errors":0,"camera_transfer_failures":0},
                 "archive_path":None,"archive_bytes":None,"error":None}
        (folder/"camera"/"frame_index.csv").write_text(
          "frame_id,frame_sequence,filename,camera_board_id,camera_monotonic_ms,camera_timestamp_utc,laptop_received_at,width,height,format,bytes,transfer_status,capture_event_id,vision_detected,vision_offset_normalized,vision_heading_error_deg,vision_confidence,vision_latency_ms\n")
        config={
          "box_id":4,"rover_id":"R04","main_controller":"ESP32","box_ssid":"CAS-Box-04","box_password":"CASbox04","box_ip":"192.168.14.1",
          "lidar":{"model":"LD20","uart":"Serial1","rx_gpio":27,"tx_gpio":33,"baud":230400,"one_degree_filtered_bins":360,
                   "dashboard_bins":90,"dashboard_bin_spacing_deg":4,"filter":"median up to three valid hits, minimum two, 500 ms expiry",
                   "raw_intensity_available":False,"native_point_frame_timestamps_available":False},
          "imu":{"model":"BNO055","address":"0x28","i2c_sda":21,"i2c_scl":22,"frame":"ISO8855 x forward y left z up",
                 "raw_vectors":["accelerometer m/s^2","gyroscope rad/s","magnetometer microtesla"],"sample_update_ms":100},
          "gps":{"model":"GY-GPS6MV2 NEO-6M","uart":"Serial2","rx_gpio":16,"tx_gpio":17,"baud":9600,
                 "library":"TinyGPSPlus","raw_nmea_retained":False},
          "ultrasonic":{"model":"HC-SR04","order":US_NAMES,"gpio_pairs":[[32,25],[4,18],[13,19],[5,14],[23,26]],
                        "max_cm":300,"filter_window":5,"timeout_us":17500},
          "drive":{"route":"laptop -> box /drive -> WebHMI -> RoverLink HTTP task -> rover",
                   "hardware_command_range":[-0.5,0.5],"laptop_survey_limit":[-MAX_DRIVE,MAX_DRIVE],
                   "wheel_encoders":False,"actual_wheel_speed_feedback":False},
          "camera":{"model":"AI-Thinker ESP32-CAM / OV2640","resolution":"QVGA 320x240 JPEG","jpeg_quality":12,"xclk_hz":20000000,"frame_buffers":1,"psram_detected_in_user_test":False,"transfer":"camera STA -> CAS-Box-04 AP -> laptop HTTP receiver",
                    "direct_to_laptop":True,"main_ESP32_relay":False},
          "sampling":{"box_sensor_snapshot_target_hz":4,"laptop_poll_target_hz":round(1/POLL_S,2),
                      "timestamps":"device millis plus laptop wall UTC and monotonic_ns"}}
        (folder/"session"/"metadata.json").write_text(json.dumps({
          "schema_version":SCHEMA,"session_id":sid,"state":"active","started_at":session["started_at"],
          "operator":operator or None,"operator_notes":notes or None,"box_url":self.box_url,
          "initial_box_state":payload.get("state"),"initial_snapshot_seq":payload["sensors"].get("snapshot_seq"),
          "start_behaviour":"Safety stop latched; rover stationary until operator explicitly enables manual drive."},indent=2))
        (folder/"session"/"configuration.json").write_text(json.dumps(config,indent=2))
        vals=values_dict(payload)
        (folder/"session"/"software_versions.json").write_text(json.dumps({
          "laptop_backend":VERSION,"dataset_schema":SCHEMA,"main_esp32":vals.get("sys.firmware","not reported"),
          "rover_firmware":"unchanged and not version-reported","camera_board_firmware":"not present in repository",
          "python":os.sys.version,"opencv":"optional; per-frame availability recorded"},indent=2))
        (folder/"session"/"timing_and_sync.json").write_text(json.dumps({
          "box_clock":"millis since boot; sample_device_ms","laptop_wall":"UTC","laptop_monotonic":"monotonic_ns",
          "clock_sync_samples":[],"warning":"Box millis is not UTC. Per-sensor samples have independent source ages and are not hardware-synchronized."},indent=2))
        self.session=session; self.estop_latched=True; self.drive_enabled=False
        self.camera_accepting=True; self.camera_uploads_inflight=0
        self.last_snapshot_seq=None; self.pending_prompt=None; self.open_area_suggestion=None
        self.last_wall_key=None; self.last_wall_prompt_mono=0; self.capture_requests=[]
        self._new_segment("Part A")
        write_jsonl(folder/"annotations"/"investigation_events.jsonl",{
          "event_id":str(uuid.uuid4()),"type":"INVESTIGATION_START","timestamp":session["started_at"],
          "session_id":sid,"operator":operator or None,"notes":notes or None})
        write_jsonl(folder/"telemetry"/"connection_events.jsonl",{
          "event_id":str(uuid.uuid4()),"type":"SESSION_STARTED_BOX_CONNECTED","timestamp":session["started_at"]})
        return self.status()

    def _new_segment(self,name):
        s=self.session; stamp=now_iso()
        segment={"segment_id":str(uuid.uuid4()),"name":name,"start_at":stamp,"end_at":None,
                 "status":"active","revision":1,"name_history":[{"name":name,"at":stamp}]}
        s.setdefault("segments",[]).append(segment)
        write_jsonl(s["dir"]/"annotations"/"segments.jsonl",{
          "event_id":str(uuid.uuid4()),"type":"SEGMENT_STARTED","timestamp":stamp,"segment":segment})
        return segment

    def segment_action(self,b):
        with self.lock:
            s=self._active(); items=s.setdefault("segments",[])
            active=next((x for x in reversed(items) if x["status"]=="active"),None)
            action=str(b.get("action","")); stamp=now_iso()
            if action=="start":
                if active: active["status"]="ended"; active["end_at"]=stamp
                return self._new_segment(str(b.get("name") or f"Part {chr(65+len(items))}"))
            if action=="rename":
                sid=b.get("segment_id") or (active or {}).get("segment_id")
                seg=next((x for x in items if x["segment_id"]==sid),None)
                name=str(b.get("name","")).strip()
                if not seg or not name: raise ValueError("Valid segment ID and name required")
                old=seg["name"]; seg["name"]=name; seg["revision"]+=1
                seg["name_history"].append({"name":name,"at":stamp,"previous_name":old})
                write_jsonl(s["dir"]/"annotations"/"segments.jsonl",{"event_id":str(uuid.uuid4()),"type":"SEGMENT_RENAMED","timestamp":stamp,"segment_id":sid,"old":old,"new":name})
                return seg
            if action in {"end","split"}:
                if not active: raise ValueError("No active segment")
                active["status"]="ended"; active["end_at"]=stamp
                write_jsonl(s["dir"]/"annotations"/"segments.jsonl",{"event_id":str(uuid.uuid4()),"type":"SEGMENT_ENDED","timestamp":stamp,"segment_id":active["segment_id"]})
                if action=="split": return self._new_segment(str(b.get("name") or f"Part {chr(65+len(items))}"))
                return active
            if action=="merge":
                ids=b.get("segment_ids") or []
                if len(ids)!=2: raise ValueError("Choose two adjacent segments")
                a=next((x for x in items if x["segment_id"]==ids[0]),None); z=next((x for x in items if x["segment_id"]==ids[1]),None)
                if not a or not z or items.index(z)!=items.index(a)+1 or a["status"]=="active" or z["status"]=="active":
                    raise ValueError("Only two adjacent ended segments can be merged")
                a["status"]=z["status"]="merged"
                merged={"segment_id":str(uuid.uuid4()),"name":str(b.get("name") or a["name"]+" + "+z["name"]),
                        "start_at":a["start_at"],"end_at":z["end_at"],"status":"merged_revision","merged_from":ids,"revision":1}
                items.append(merged)
                write_jsonl(s["dir"]/"annotations"/"segments.jsonl",{"event_id":str(uuid.uuid4()),"type":"SEGMENTS_MERGED","timestamp":stamp,"merged_segment":merged})
                return merged
            raise ValueError("Unknown segment action")

    def _active(self):
        if not self.session or self.session.get("state")!="active": raise RuntimeError("No active investigation")
        return self.session

    def annotate(self,b):
        with self.lock:
            s=self._active(); tags=[str(x) for x in b.get("tags",[])]
            bad=[x for x in tags if x not in TAGS]
            if bad: raise ValueError("Unknown tag(s): "+", ".join(bad))
            seg=next((x for x in reversed(s.get("segments",[])) if x["status"]=="active"),None)
            snap=(self.latest or {}).get("sensors") or {}
            rec={"annotation_id":str(uuid.uuid4()),"timestamp":now_iso(),"session_id":s["id"],
                 "segment_id":b.get("segment_id") or (seg or {}).get("segment_id"),
                 "segment_name_at_annotation":(seg or {}).get("name"),"tags":tags,
                 "notes":str(b.get("notes","")).strip() or None,"object_type":b.get("object_type"),
                 "object_side":b.get("object_side"),"opposite_context":b.get("opposite_context"),
                 "pillar_count_reliable":b.get("pillar_count_reliable"),
                 "vision_review_requested":bool(b.get("vision_review_requested",False)),
                 "box_snapshot_sequence":snap.get("snapshot_seq"),"box_device_ms":snap.get("snapshot_device_ms"),
                 "latest_camera_frame_id":s.get("latest_frame_id")}
            write_jsonl(s["dir"]/"annotations"/"annotations.jsonl",rec)
            write_jsonl(s["dir"]/"annotations"/"investigation_events.jsonl",{"event_id":str(uuid.uuid4()),"type":"OPERATOR_ANNOTATION",**rec})
            return rec

    def capture(self,notes=""):
        with self.lock:
            s=self._active(); snap=(self.latest or {}).get("sensors") or {}; seg=next((x for x in reversed(s.get("segments",[])) if x["status"]=="active"),None)
            eid=str(uuid.uuid4()); stamp=now_iso()
            rec={"event_id":eid,"type":"INVESTIGATION_CAPTURE","timestamp":stamp,"session_id":s["id"],
                 "segment_id":(seg or {}).get("segment_id"),"box_snapshot_sequence":snap.get("snapshot_seq"),
                 "box_device_ms":snap.get("snapshot_device_ms"),"lidar_age_ms":(snap.get("lidar") or {}).get("age_ms"),
                 "imu_age_ms":(snap.get("imu") or {}).get("age_ms"),"gps_age_ms":(snap.get("gps") or {}).get("age_ms"),
                 "ultrasonic_age_ms":snap.get("usAgeMs"),"notes":notes or None,"camera_requested":True}
            write_jsonl(s["dir"]/"annotations"/"investigation_events.jsonl",rec)
            self.capture_requests.append({"event_id":eid,"frames_remaining":3,"requested_at":stamp})
            return rec

    def stop_rover(self,reason="operator_stop"):
        with self.lock:
            self.estop_latched=True; self.drive_enabled=False; s=self.session
            if s and s.get("state")=="active":
                write_jsonl(s["dir"]/"annotations"/"investigation_events.jsonl",{
                  "event_id":str(uuid.uuid4()),"type":"ROVER_STOP_REQUESTED","timestamp":now_iso(),
                  "reason":reason,"logging_continues":True})
        ok=self._send_estop()
        if s and s.get("state")=="active":
            write_jsonl(s["dir"]/"telemetry"/"connection_events.jsonl",{
              "event_id":str(uuid.uuid4()),"type":"BOX_STOP_ACK" if ok else "BOX_STOP_FAILED","timestamp":now_iso(),"reason":reason})
        return {"ok":ok,"drive_inhibited":True,"logging_continues":bool(s and s.get("state")=="active"),
                "message":"Box acknowledged latched stop" if ok else "Box did not acknowledge stop; network deadman may still stop after command expiry. Check connection."}

    def _send_estop(self):
        try: return self.box_request("POST","/safety/stop",timeout=.8).status_code==200
        except Exception: return False

    def enable_manual_drive(self):
        with self.lock: self._active()
        r=self.box_request("POST","/safety/clear",timeout=.8); r.raise_for_status()
        with self.lock:
            self.estop_latched=False; self.drive_enabled=True
            write_jsonl(self.session["dir"]/"annotations"/"investigation_events.jsonl",{
              "event_id":str(uuid.uuid4()),"type":"MANUAL_DRIVE_ENABLED","timestamp":now_iso(),"explicit_operator_action":True})
        return {"ok":True,"drive_enabled":True,"motors_remain_zero_until_commanded":True}

    def drive(self,left,right,source="laptop_web_ui"):
        with self.lock:
            s=self._active()
            if self.estop_latched or not self.drive_enabled: raise PermissionError("Clear stop latch and explicitly enable manual drive first")
            left=max(-MAX_DRIVE,min(MAX_DRIVE,float(left))); right=max(-MAX_DRIVE,min(MAX_DRIVE,float(right)))
            self.drive_sequence+=1; seq=self.drive_sequence; pid=self.drive_page_id
        try:
            r=self.box_request("GET","/drive",params={"l":f"{left:.3f}","r":f"{right:.3f}","id":pid,"n":seq},timeout=.6)
            ok=r.status_code==200 and bool(r.json().get("ok")); reply=r.text[:200]
        except Exception as exc: ok=False; reply=f"{type(exc).__name__}: {exc}"
        stamp=now_iso()
        write_csv(s["dir"]/"telemetry"/"motor_commands.csv",
          ["session_id","timestamp","source","collector_seq","requested_left","requested_right","box_reported_left","box_reported_right","command_age_ms","rover_voltage_v","rover_rtt_ms"],
          [s["id"],stamp,source,None,left,right,None,None,None,None,None])
        write_jsonl(s["dir"]/"annotations"/"investigation_events.jsonl",{
          "event_id":str(uuid.uuid4()),"type":"MANUAL_DRIVE_COMMAND","timestamp":stamp,"source":source,
          "sequence":seq,"requested_left":left,"requested_right":right,"box_acknowledged":ok,
          "meaning":"command fraction; not measured wheel speed"})
        if not ok: s["quality"]["drive_command_transfer_failures"]=s["quality"].get("drive_command_transfer_failures",0)+1
        return {"ok":ok,"left":left,"right":right,"sequence":seq,"box_reply":reply}

    def _analyse_navigation(self,payload):
        if not self.session or self.session.get("state")!="active": return
        result=analyse_lidar(payload.get("sensors"))
        self.session["latest_geometry_analysis"]=result
        walls=result.get("possible_walls") or []
        if walls and not self.pending_prompt:
            best=max(walls,key=lambda x:x["confidence"]); key=str(best["side"]); now=time.monotonic()
            if key!=self.last_wall_key or now-self.last_wall_prompt_mono>20:
                self.pending_prompt={"prompt_id":str(uuid.uuid4()),"kind":"possible_wall",
                  "side":"both" if len(walls)==2 else key,"confidence":max(x["confidence"] for x in walls),
                  "evidence":walls,"created_at":now_iso(),
                  "message":"Possible planar wall geometry; operator must inspect. This is not semantic object recognition."}
                self.last_wall_key=key; self.last_wall_prompt_mono=now
                write_jsonl(self.session["dir"]/"telemetry"/"navigation_events.jsonl",{
                  "timestamp":self.pending_wall_prompt_timestamp(),"type":"POSSIBLE_WALL_PROMPT",
                  "prompt":self.pending_prompt,"motion_authorized":False})
        self.open_area_suggestion=result.get("centreline") or result.get("open_area_prompt")

    def pending_wall_prompt_timestamp(self):
        return self.pending_prompt.get("created_at") if self.pending_prompt else now_iso()

    def wall_decision(self,b):
        with self.lock:
            s=self._active()
            if not self.pending_prompt: raise ValueError("No pending possible-wall prompt")
            decision=str(b.get("decision",""))
            if decision not in {"approve","ignore","capture"}: raise ValueError("Decision must be approve, ignore or capture")
            p=self.pending_prompt
            rec={"event_id":str(uuid.uuid4()),"type":"WALL_PROMPT_DECISION","timestamp":now_iso(),
                 "prompt_id":p["prompt_id"],"decision":decision,"side":p["side"],"confidence":p["confidence"],
                 "evidence":p["evidence"],"wall_following_state":"approved_but_motion_disabled" if decision=="approve" else None,
                 "motion_authorized":False,
                 "reason":"No physically calibrated wall-follow controller is enabled; approval is logged only." if decision=="approve" else None}
            if decision=="capture":
                self.capture_requests.append({"event_id":rec["event_id"],"frames_remaining":3,"requested_at":rec["timestamp"]})
            write_jsonl(s["dir"]/"annotations"/"investigation_events.jsonl",rec)
            self.pending_prompt=None
            return rec

    def centreline_decision(self,b):
        with self.lock:
            s=self._active(); suggestion=self.open_area_suggestion
            if not suggestion: raise ValueError("No current centreline/open-area suggestion")
            decision=str(b.get("decision",""))
            if decision not in {"confirm","reject","capture","manual_continue"}: raise ValueError("Invalid decision")
            rec={"event_id":str(uuid.uuid4()),"type":"CENTRELINE_OPERATOR_DECISION","timestamp":now_iso(),
                 "decision":decision,"suggestion":suggestion,"motion_authorized":False,
                 "note":"This action stores survey metadata only; no autonomous drive is started."}
            write_jsonl(s["dir"]/"annotations"/"investigation_events.jsonl",rec)
            self.open_area_suggestion=None
            if decision=="capture": self.capture_requests.append({"event_id":rec["event_id"],"frames_remaining":3,"requested_at":rec["timestamp"]})
            return rec

    def camera_session(self):
        with self.lock:
            s=self.session
            if not s or s["state"]!="active" or not self.camera_accepting:
                return {"active":False,"session_id":None,"capture":None}
            cap=self.capture_requests[0] if self.capture_requests else None
            return {"active":True,"session_id":s["id"],"capture":cap}

    def begin_camera_upload(self, session_id):
        with self.camera_upload_condition:
            if (not self.camera_accepting or not self.session or
                self.session.get("state")!="active" or self.session.get("id")!=session_id):
                return False
            self.camera_uploads_inflight += 1
            return True

    def end_camera_upload(self):
        with self.camera_upload_condition:
            self.camera_uploads_inflight=max(0,self.camera_uploads_inflight-1)
            self.camera_upload_condition.notify_all()

    def camera_saved(self,meta,path):
        with self.lock:
            s=self._active(); seq=meta.get("frame_sequence")
            if isinstance(seq,int) and s["camera_sequence_last"] is not None and seq>s["camera_sequence_last"]+1:
                s["camera_image_gaps"]+=seq-s["camera_sequence_last"]-1
            if isinstance(seq,int): s["camera_sequence_last"]=seq
            s["camera_frame_count"]+=1; s["latest_frame_id"]=meta["frame_id"]
            self.camera_frames_received+=1; self.camera_missing_frames+=max(0, seq-(s.get("camera_sequence_prev",seq) or seq)-1) if isinstance(seq,int) else 0; s["camera_sequence_prev"]=seq if isinstance(seq,int) else s.get("camera_sequence_prev"); self.camera_last_frame_at=meta["laptop_received_at"]; self.camera_last_frame_mono=time.monotonic()
            eid=meta.get("capture_event_id")
            if eid:
                for q in list(self.capture_requests):
                    if q["event_id"]==eid:
                        q["frames_remaining"]-=1
                        if q["frames_remaining"]<=0: self.capture_requests.remove(q)
            write_csv(s["dir"]/"camera"/"frame_index.csv",
              ["frame_id","frame_sequence","filename","camera_board_id","camera_monotonic_ms","camera_timestamp_utc",
               "laptop_received_at","width","height","format","bytes","transfer_status","capture_event_id",
               "vision_detected","vision_offset_normalized","vision_heading_error_deg","vision_confidence","vision_latency_ms"],
              [meta["frame_id"],seq,str(path.relative_to(s["dir"])),meta.get("camera_board_id"),meta.get("camera_monotonic_ms"),
               meta.get("camera_timestamp_utc"),meta["laptop_received_at"],meta.get("width"),meta.get("height"),"jpeg",
               meta["bytes"],"stored",eid,None,None,None,None,None])
            write_jsonl(s["dir"]/"annotations"/"investigation_events.jsonl",{
              "event_id":str(uuid.uuid4()),"type":"CAMERA_FRAME_STORED","timestamp":meta["laptop_received_at"],
              "frame_id":meta["frame_id"],"capture_event_id":eid,"filename":str(path.relative_to(s["dir"]))})
        return meta

    def set_vision(self,frame_id,result):
        with self.lock:
            self.last_vision={"frame_id":frame_id,**result}
            if self.session and self.session["state"]=="active":
                stamp=now_iso()
                write_jsonl(self.session["dir"]/"derived"/"vision_tracking.jsonl",{
                  "timestamp":stamp,"frame_id":frame_id,**result})
                write_csv(self.session["dir"]/"derived"/"vision_tracking.csv",
                  ["timestamp","frame_id","available","valid","classification","detected","candidate_rib_count",
                   "lateral_offset_normalized","heading_error_image_deg","confidence","processing_latency_ms","reason"],
                  [stamp,frame_id,result.get("available"),result.get("valid"),result.get("classification"),
                   result.get("detected"),result.get("candidate_rib_count"),result.get("lateral_offset_normalized"),
                   result.get("heading_error_image_deg"),result.get("confidence"),result.get("processing_latency_ms"),
                   result.get("reason")])

    def finalize(self):
        # Stop rover first; camera is not awaited.
        stop=self.stop_rover("investigation_finalize")
        with self.camera_upload_condition:
            s=self.session
            if not s: raise RuntimeError("No investigation to finalize")
            if s["state"]=="ready": return self.status()
            if s["state"] not in {"active","finalize_error","storage_error"}: raise RuntimeError(f"Cannot finalize from {s['state']}")
            self.camera_accepting=False
            deadline=time.monotonic()+2.5
            while self.camera_uploads_inflight>0 and time.monotonic()<deadline:
                self.camera_upload_condition.wait(timeout=max(0.01,deadline-time.monotonic()))
            pending_uploads=self.camera_uploads_inflight
            s["quality"]["camera_uploads_inflight_at_finalize"]=pending_uploads
            if pending_uploads:
                s["quality"]["camera_uploads_not_completed_at_finalize"]=pending_uploads
                write_jsonl(s["dir"]/"telemetry"/"connection_events.jsonl",{
                  "event_id":str(uuid.uuid4()),"type":"CAMERA_UPLOAD_DRAIN_TIMEOUT","timestamp":now_iso(),
                  "inflight_uploads":pending_uploads,"note":"Upload not committed before finalization timeout."})
            if self.capture_requests:
                s["quality"]["camera_capture_bursts_incomplete"]=len(self.capture_requests)
                for pending in self.capture_requests:
                    write_jsonl(s["dir"]/"annotations"/"investigation_events.jsonl",{
                      "event_id":str(uuid.uuid4()),"type":"CAMERA_CAPTURE_INCOMPLETE_AT_FINALIZE",
                      "timestamp":now_iso(),"capture_event_id":pending.get("event_id"),
                      "frames_remaining":pending.get("frames_remaining")})
                self.capture_requests=[]
            s["state"]="finalizing"; s["finalize_stop_acknowledged"]=stop["ok"]
            stamp=now_iso()
            write_jsonl(s["dir"]/"annotations"/"investigation_events.jsonl",{
              "event_id":str(uuid.uuid4()),"type":"FINALIZATION_STARTED","timestamp":stamp,"box_stop_acknowledged":stop["ok"]})
            active=next((x for x in reversed(s.get("segments",[])) if x["status"]=="active"),None)
            if active:
                active["status"]="ended"; active["end_at"]=stamp
                write_jsonl(s["dir"]/"annotations"/"segments.jsonl",{"event_id":str(uuid.uuid4()),"type":"SEGMENT_ENDED_AT_FINALIZE","timestamp":stamp,"segment_id":active["segment_id"]})
        try:
            folder=s["dir"]
            metadata={"schema_version":SCHEMA,"session_id":s["id"],"state":"ready_for_review",
              "started_at":s["started_at"],"finalized_at":now_iso(),"operator":s["operator"] or None,
              "operator_notes":s["notes"] or None,"box_url":self.box_url,
              "sample_count":s["quality"]["sensor_snapshots"],"camera_frames":s["camera_frame_count"],
              "box_stop_acknowledged":stop["ok"],
              "limitations":["No wheel encoder interface is present; commanded motor fractions are not measured speed/distance.",
                "LD20 raw intensity and native per-point timestamps are not exposed by the current parser.",
                "BNO055 raw vectors are updated at ~10 Hz, then latest values are included in ~4 Hz snapshots.",
                "TinyGPSPlus raw NMEA sentences are not retained; decoded fields only.",
                "Camera data exists only if ESP32-S3 is configured to join CAS-Box-07 and HTTP POST directly to laptop.",
                "Wall/centreline/tactile-strip results are non-actuating candidate observations."] }
            (folder/"session"/"metadata.json").write_text(json.dumps(metadata,indent=2),encoding="utf-8")
            (folder/"session"/"timing_and_sync.json").write_text(json.dumps({
              "box_clock":"millis since boot; not UTC","laptop_wall_clock":"UTC","laptop_monotonic_clock":"monotonic_ns",
              "sync_samples":self.clock_samples,"warning":"Sensors are acquired on separate schedules, not hardware-synchronized."},indent=2),encoding="utf-8")
            (folder/"session"/"segments_state.json").write_text(json.dumps(s.get("segments",[]),indent=2),encoding="utf-8")
            quality=dict(s["quality"])
            quality.update({"session_id":s["id"],"finalized_at":now_iso(),
              "box_poll_failures_during_session":max(0,self.total_poll_failures-s["poll_failures_at_start"]),
              "camera_frames_received":s["camera_frame_count"],"camera_sequence_gaps":s["camera_image_gaps"],
              "box_stop_acknowledged":stop["ok"],
              "unavailable_fields":{"lidar_intensity":"not exposed","lidar_native_point_timing":"not exposed",
                                     "gps_raw_nmea":"not retained","wheel_speed":"no encoders/code",
                                     "camera_images":"only if camera board uploads"}})
            (folder/"integrity"/"data_quality_report.json").write_text(json.dumps(quality,indent=2),encoding="utf-8")
            (folder/"README.md").write_text(self._readme(s,quality),encoding="utf-8")
            checksums={}
            for p in sorted(folder.rglob("*")):
                if p.is_file() and not p.name.startswith(".") and not p.name.endswith(".part") and p.name not in {"checksums.json","manifest.json"}:
                    h=hashlib.sha256()
                    with p.open("rb") as f:
                        for chunk in iter(lambda:f.read(1024*1024),b""): h.update(chunk)
                    checksums[str(p.relative_to(folder))]={"sha256":h.hexdigest(),"bytes":p.stat().st_size}
            (folder/"integrity"/"checksums.json").write_text(json.dumps(checksums,indent=2),encoding="utf-8")
            (folder/"manifest.json").write_text(json.dumps({
              "schema_version":SCHEMA,"session_id":s["id"],"generated_at":now_iso(),
              "file_count":len(checksums)+2,"files":checksums,"checksums_file":"integrity/checksums.json"},indent=2),encoding="utf-8")
            archive=self.archives_root/f"CAS_B3_R04_Survey_{s['id']}.zip"
            if archive.exists(): archive.unlink()
            with zipfile.ZipFile(archive,"w",compression=zipfile.ZIP_DEFLATED,compresslevel=4) as z:
                for p in sorted(folder.rglob("*")):
                    if p.is_file() and not p.name.startswith(".") and not p.name.endswith(".part"):
                        z.write(p,arcname=str(Path(s["id"])/p.relative_to(folder)))
            with zipfile.ZipFile(archive) as z:
                bad=z.testzip()
                if bad: raise RuntimeError(f"ZIP CRC check failed for {bad}")
                manifest=json.loads(z.read(f"{s['id']}/manifest.json"))
                if manifest.get("session_id")!=s["id"]: raise RuntimeError("Manifest session mismatch")
                for rel, expected in manifest.get("files", {}).items():
                    arc_name=str(Path(s["id"])/rel)
                    digest=hashlib.sha256()
                    total=0
                    with z.open(arc_name, "r") as item:
                        for chunk in iter(lambda: item.read(1024*1024), b""):
                            digest.update(chunk)
                            total += len(chunk)
                    if digest.hexdigest()!=expected.get("sha256") or total!=expected.get("bytes"):
                        raise RuntimeError(f"Checksum/size mismatch after ZIP creation: {rel}")
            with self.lock:
                s["archive_path"]=archive; s["archive_bytes"]=archive.stat().st_size
                s["state"]="ready"; s["finalized_at"]=now_iso(); s["error"]=None
                return self.status()
        except Exception as exc:
            with self.lock:
                s["state"]="finalize_error"; s["error"]=f"{type(exc).__name__}: {exc}"
            raise

    def _readme(self,s,q):
        return f"""CAS Box B3/R04 survey dataset
Session: {s['id']}
Started: {s['started_at']}
Finalized: {now_iso()}
Schema: {SCHEMA}

Contains timestamped laptop-received sensor snapshots, filtered LiDAR scans, IMU/GPS/ultrasonic CSV,
motor command requests, system diagnostics, connection events, camera frames/index where available,
operator annotations, route-segment history, analysis outputs, quality report and SHA-256 checksums.

Clock domains: ESP32 millis since boot; laptop UTC and monotonic_ns. Samples are not hardware-synchronized.
No encoder measurements exist. Motor commands do not prove wheel speed/distance. LD20 raw intensity,
native frame-point timestamps and GPS raw NMEA are not exposed by the current firmware. See
integrity/data_quality_report.json before using this data for training/navigation analysis.
"""

    def status(self):
        with self.lock:
            s=self.session; active=bool(s and s["state"] in {"active","finalizing","finalize_error","storage_error"})
            disk=shutil.disk_usage(self.data_root)
            camera_age=time.monotonic()-self.camera_last_frame_mono if self.camera_last_frame_mono else None
            sinfo=None
            if s:
                sinfo={"session_id":s["id"],"state":s["state"],"started_at":s["started_at"],
                  "samples":s["quality"]["sensor_snapshots"],"collector_sequence":s["collector_seq"],
                  "images":s["camera_frame_count"],"error":s.get("error"),"archive_bytes":s.get("archive_bytes"),"segments":s.get("segments",[]),
                  "download_url":f"/api/session/download/{s['id']}" if s["state"]=="ready" else None}
            return {"schema_version":SCHEMA,"backend_version":VERSION,"box_url":self.box_url,
              "box_online":self.box_online,"box_last_receive_at":self.box_last_receive_at,
              "box_last_receive_monotonic_ns":self.box_last_receive_mono_ns,
              "box_last_error":self.box_last_error,"box_poll_failures_current":self.poll_failures,
              "box_poll_failures_total":self.total_poll_failures,"latest":self.latest,
              "session_active":active,"session":sinfo,"drive_enabled":self.drive_enabled,
              "estop_latched":self.estop_latched,"pending_wall_prompt":self.pending_prompt,
              "centreline_suggestion":self.open_area_suggestion,"wall_following_motion_enabled":False,
              "wall_following_disabled_reason":"No calibrated/physically validated wall-follow motion controller is enabled.",
              "last_vision":self.last_vision,
              "camera":{"last_frame_at":self.camera_last_frame_at,"last_frame_age_s":None if camera_age is None else round(camera_age,2),
                "frames_received_total":self.camera_frames_received,"transfer_errors":self.camera_errors,
                "sequence_gaps":self.camera_missing_frames,"online":camera_age is not None and camera_age<5,
                "token_configured":bool(self.camera_token and self.camera_token!="CHANGE_ME_CAMERA_TOKEN")},
              "storage":{"path":str(self.data_root),"free_bytes":disk.free,"total_bytes":disk.total}}

    def archive(self,sid):
        with self.lock:
            if self.session and self.session["id"]==sid and self.session["state"]=="ready":
                return Path(self.session["archive_path"])
        matches=list(self.archives_root.glob("CAS_B3_R04_Survey_"+sid+".zip"))
        if len(matches)==1 and matches[0].is_file():
            return matches[0]
        raise FileNotFoundError("Validated ZIP not found for this session")
