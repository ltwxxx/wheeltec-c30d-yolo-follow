from http.client import HTTPConnection, HTTPException
from io import BytesIO
import json
import logging
import os
from pathlib import Path
import secrets
import subprocess
from collections import deque
from follow_session import FollowSession
from follow_settings import SETTINGS, VERSION
import threading
import time

from flask import Flask, Response, jsonify, render_template, request

from follow_control import (ARRIVAL_DISTANCE_M, AUTO_MAX_FORWARD_MPS, FRAME_TIMEOUT, MAX_FORWARD_MPS,
                            fetch_frame, fresh_lidar, observe_person)

app = Flask(__name__)
app.config["TRUSTED_HOSTS"] = ["127.0.0.1", "localhost"]
app.config["MAX_CONTENT_LENGTH"] = 4096
logging.getLogger("werkzeug").setLevel(logging.ERROR)
EDGE_MODEL = os.environ.get('WHEELTEC_EDGE_MODEL')
VIEW_ONLY = os.environ.get('WHEELTEC_VIEW_ONLY') == '1'
VIEWER_PORT = int(os.environ.get('WHEELTEC_VIEWER_PORT', '8769'))
if EDGE_MODEL:
    from edge_detector import Detector
    from edge_tracking import EdgeTracker, draw_tracks
else:
    import cv2
    import torch
    from ultralytics import YOLO

    DEVICE = 'mps' if torch.backends.mps.is_available() else 'cpu'
    torch.set_num_threads(2)
    torch.set_num_interop_threads(1)
    cv2.setNumThreads(1)

DISPLAY_RESULT_TIMEOUT = 1.25
TRACKER_CONFIG = str(Path(__file__).with_name('botsort_reid.yaml'))
lock = threading.Lock()
observations_changed = threading.Condition(lock)
motion_lock = threading.Lock()
state = {'enabled': bool(EDGE_MODEL), 'status': '正在连接相机', 'frame': None,
         'ids': [], 'generation': 0, 'detect_ms': None}
session = FollowSession()
follow = session.data
performance = deque(maxlen=600)
control_performance = deque(maxlen=600)
request_cache = {}
shutting_down = False
observations = {}
observation_time = 0.0
observation_sensors = None
WEIGHTS = os.environ.get('WHEELTEC_YOLO_WEIGHTS', '/private/tmp/wheeltec-yolo11n.pt')
latest = threading.Condition()
raw = None
sequence = 0
capture_epoch = 0
SENSOR_ERRORS = (OSError, HTTPException, ValueError, KeyError, TypeError)
CONTROL_URL = "127.0.0.1"
CONTROL_PORT = 18768
CONTROL_TIMEOUT = 0.22
CONTROL_PERIOD_S = 0.05
EDGE_CAPTURE_PERIOD_S = 0.05
EDGE_PREVIEW_PERIOD_S = 0.25
START_SESSION = secrets.token_hex(8)
AUTO_TARGET = 'auto'
UNLOCKED_TARGET = 'unlocked'
AUTO_TARGETS = (AUTO_TARGET, UNLOCKED_TARGET)
LOCAL_TRIAL_SECONDS = 10
LOCAL_TRIAL_FORWARD_MPS = 0.07
_control_http = None
_control_http_lock = threading.Lock()
control_link = {'ready': False, 'subscribers': 0, 'battery_voltage': None,
                'fast_forward': False, 'error': '正在连接底盘控制桥', 'available_forward_mps': 0., 'battery': {}, 'sent': {}, 'odometry': None}
last_page_heartbeat = 0.0
page_heartbeats = {}
page_actions = {}
start_generation = 0


@app.before_request
def check_local_request():
    if request.host.split(':', 1)[0] not in ('127.0.0.1', 'localhost'):
        return {'error': '请从本机控制台访问'}, 400
    origin = request.headers.get('Origin')
    if origin and origin != f'http://{request.host}':
        return {'error': '请求来源无效'}, 403


@app.after_request
def protect_console(response):
    response.headers['Cache-Control'] = 'no-store'
    response.headers['X-Frame-Options'] = 'DENY'
    return response


def actuator_request(method, path, payload=None):
    global _control_http
    body = json.dumps(payload).encode() if payload is not None else None
    headers = {'Content-Type': 'application/json'} if body is not None else {}
    with _control_http_lock:
        if _control_http is None:
            _control_http = HTTPConnection(CONTROL_URL, CONTROL_PORT, timeout=CONTROL_TIMEOUT)
        try:
            _control_http.request(method, path, body=body, headers=headers)
            response = _control_http.getresponse()
            data = json.loads(response.read(4096))
            if response.status != 200:
                raise RuntimeError(data.get('error', f'控制桥 HTTP {response.status}'))
            return data
        except (OSError, HTTPException, ValueError, RuntimeError):
            _control_http.close()
            _control_http = None
            raise


def publish_actuator_command(active, forward=0.0, turn=0.0):
    return actuator_request('POST', '/command', {
        'active': active, 'forward_mps': forward, 'turn_rad_s': turn,
    })


def control_loop():
    next_health = 0.0
    last_measured_frame = 0.0
    while True:
        tick = time.monotonic()
        if tick >= next_health:
            try:
                health = actuator_request('GET', '/health')
                with lock:
                    control_link.update(health)
                    control_link.update(ready=bool(health.get('ok')), error='', health_at=time.monotonic())
            except (OSError, HTTPException, ValueError, RuntimeError) as exc:
                with lock:
                    control_link.update(ready=False, error=f'底盘控制桥不可用：{exc}')
            next_health = tick + .5
        with motion_lock:
            with lock:
                now = time.monotonic()
                try:
                    session.update(observations, observation_sensors, observation_time, now,
                                   health_view(now), enabled=state['enabled'] and not state.get('inference_error', False))
                except Exception:
                    logging.getLogger(__name__).exception('Control update failed')
                    session._wait('fault_paused', '控制计算异常，等待有效传感器数据', now)
                forward, turn = follow['forward_mps'], follow['turn_rad_s']
                active = bool(session.armed and (forward or turn) and not VIEW_ONLY)
            if not VIEW_ONLY:
                try:
                    publish_actuator_command(active, forward if active else 0., turn if active else 0.)
                    with lock:
                        session.sent(forward if active else 0., turn if active else 0.)
                except (OSError, HTTPException, ValueError, RuntimeError) as exc:
                    with lock:
                        control_link.update(ready=False, error=f'底盘控制通道中断：{exc}')
                        session.sent(0., 0.)
            with lock:
                state['control_tick_at'] = time.monotonic()
                if observation_time and observation_time != last_measured_frame:
                    control_performance.append((state['control_tick_at'], state['control_tick_at'] - observation_time))
                    last_measured_frame = observation_time
        time.sleep(max(.005, CONTROL_PERIOD_S - (time.monotonic() - tick)))


def follow_armed():
    return session.armed


def health_view(now):
    view = dict(control_link)
    if view.get('odometry'):
        view['odometry'] = dict(view['odometry'])
        view['odometry']['age_s'] += max(0., now - view.get('health_at', now))
    return view


def stop_follow(reason, distance=None):
    session.stop(time.monotonic(), reason)


def publish_frame(image, captured_at, sensors):
    global raw, sequence
    with latest:
        raw = (image, captured_at, sensors)
        sequence += 1
        latest.notify()


def disconnect(reason):
    global raw, capture_epoch, observation_sensors
    with latest:
        raw = None
        capture_epoch += 1
        with lock:
            state.update(status=reason, frame=None, ids=[])
            observations.clear()
            observation_sensors = None
            observations_changed.notify_all()


def capture():
    last_stamp = None
    last_received = 0.0
    misses = 0
    failure_status = '相机数据桥连接失败'
    while True:
        tick = time.monotonic()
        try:
            packet = fetch_frame(as_pil=bool(EDGE_MODEL), after_stamp=last_stamp if EDGE_MODEL else None,
                                 allow_color_only=bool(EDGE_MODEL))
            if packet is None or (last_stamp is not None and packet[0] <= last_stamp):
                misses += 1
                failure_status = '相机画面没有更新，正在重连'
            else:
                stamp, image, captured_at, sensors = packet
                publish_frame(image, captured_at, sensors)
                with lock:
                    state['capture_fetch_ms'] = round((time.monotonic() - tick) * 1000, 1)
                    state['capture_age_ms'] = round((time.monotonic() - captured_at) * 1000, 1)
                last_stamp = stamp
                last_received = time.monotonic()
                misses = 0
        except SENSOR_ERRORS:
            misses += 1
            failure_status = '相机或深度暂不可用'
        now = time.monotonic()
        if misses >= 3 and (not EDGE_MODEL or now - last_received >= FRAME_TIMEOUT):
            disconnect(failure_status)
            last_stamp = None
            misses = 0
        period = EDGE_CAPTURE_PERIOD_S if EDGE_MODEL else 0.1
        time.sleep(max(0.02 if EDGE_MODEL else 0.03, period - (now - tick)))


def worker():
    global observation_time, observation_sensors
    model = None
    tracker = None
    seen = 0
    previous_generation = None
    next_preview = 0.0
    while True:
        with latest:
            latest.wait_for(lambda: raw is not None and sequence != seen)
            (frame, frame_time, sensors), seen, epoch = raw, sequence, capture_epoch
        with lock:
            enabled, generation = state['enabled'], state['generation']
            use_depth = not follow['unlocked']
            state['worker_start_age_ms'] = round((time.monotonic() - frame_time) * 1000, 1)
        ids = []
        tracked_people = {}
        boxes = {}
        visible_boxes = {}
        detect_ms = None
        status = '识别已关闭'
        failed = False
        if enabled:
            try:
                if EDGE_MODEL and tracker is None:
                    with lock:
                        state['status'] = '正在加载车端模型'
                    tracker = EdgeTracker(Detector(EDGE_MODEL))
                elif not EDGE_MODEL and model is None:
                    with lock:
                        state['status'] = '正在加载模型'
                    model = YOLO(WEIGHTS)
                session = generation if EDGE_MODEL else (generation, epoch)
                if session != previous_generation:
                    if EDGE_MODEL:
                        tracker.reset()
                    elif model.predictor is not None:
                        for previous_tracker in getattr(model.predictor, 'trackers', ()):
                            previous_tracker.reset()
                previous_generation = session
                if EDGE_MODEL:
                    started = time.monotonic()
                    with lock:
                        selected_id, locked_mode = follow['target'], not follow['unlocked']
                    boxes = tracker.track(frame, frame_time, selected_id=selected_id, locked=locked_mode)
                    detect_ms = round((time.monotonic() - started) * 1000, 1)
                    ids = tracker.known_ids
                    visible_boxes = tracker.visible_boxes
                    for track_id, box in boxes.items():
                        left, top, right, bottom = box
                        tracked_people[track_id] = observe_person((left / frame.width, top / frame.height,
                                                                   right / frame.width, bottom / frame.height),
                                                                  sensors, use_depth=use_depth)
                    status = f'车端识别中 · {len(boxes)} 人'
                else:
                    result = model.track(frame, persist=True, tracker=TRACKER_CONFIG, classes=[0],
                                         imgsz=480, verbose=False, device=DEVICE)[0]
                    if result.boxes.id is not None:
                        ids = result.boxes.id.int().tolist()
                        tracked_people = {track_id: observe_person(box, sensors)
                                          for track_id, box in zip(ids, result.boxes.xyxyn.tolist())}
                    frame = result.plot()
                    status = f'识别中 · {len(result.boxes)} 人'
                if ids:
                    status += ' · 跟踪编号 ' + ', '.join(map(str, ids))
            except Exception as exc:
                logging.getLogger(__name__).exception('Person tracking failed')
                status = f'模型错误：{exc}'
                failed = True
        with latest:
            if raw is None or epoch != capture_epoch:
                continue
            with lock:
                if generation != state['generation']:
                    continue
                state.update(status=status, ids=ids, detect_ms=detect_ms)
                state['worker_end_age_ms'] = round((time.monotonic() - frame_time) * 1000, 1)
                observations.clear()
                observations.update(tracked_people)
                observation_time = frame_time
                observation_sensors = sensors
                if not failed:
                    performance.append((time.monotonic(), time.monotonic() - frame_time))
                state['inference_error'] = failed
                state['reid'] = tracker.reid_status if EDGE_MODEL and tracker else {}
                selected = follow['target']
                observations_changed.notify_all()
        if failed and EDGE_MODEL:
            time.sleep(0.25)
        if EDGE_MODEL:
            now = time.monotonic()
            if now < next_preview:
                continue
            next_preview = now + EDGE_PREVIEW_PERIOD_S
            output = BytesIO()
            if selected in visible_boxes:
                boxes[selected] = visible_boxes[selected]
            draw_tracks(frame, boxes, selected).save(output, format='JPEG', quality=48)
            jpeg = output.getvalue()
        else:
            ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if not ok:
                continue
            jpeg = encoded.tobytes()
        with latest:
            with lock:
                if epoch == capture_epoch and generation == state['generation']:
                    state['frame'] = jpeg


@app.get('/')
def home():
    return render_template('follow.html', max_speed=AUTO_MAX_FORWARD_MPS if EDGE_MODEL else MAX_FORWARD_MPS,
                           low_speed=MAX_FORWARD_MPS,
                           follow_distance=ARRIVAL_DISTANCE_M, edge=bool(EDGE_MODEL),
                           trial_seconds=LOCAL_TRIAL_SECONDS, trial_speed=LOCAL_TRIAL_FORWARD_MPS)


@app.post('/detection')
def detection():
    global start_generation
    data = request.get_json(silent=True) or {}
    if not isinstance(data, dict) or type(data.get('enabled')) is not bool:
        return {'error': 'invalid request'}, 400
    with motion_lock, lock:
        if follow['unlocked'] and not data['enabled']:
            return {'error': '请先点击停止不认主跟随'}, 409
        if state['enabled'] != data['enabled']:
            state.update(enabled=data['enabled'], ids=[], frame=None)
            state['generation'] += 1
            state['status'] = '正在启动识别' if data['enabled'] else '识别已关闭'
        if not data['enabled']:
            observations.clear()
            stop_follow('已停止：识别关闭')
            start_generation += 1
            observations_changed.notify_all()
    return {'ok': True}


@app.post('/follow')
def set_follow():
    global start_generation
    data = request.get_json(silent=True)
    if not isinstance(data, dict) or 'target' not in data:
        return {'error': '请选择人物编号'}, 400
    target = data['target']
    if target is not None and target not in AUTO_TARGETS and (type(target) is not int or target < 1):
        return {'error': '人物编号无效'}, 400
    trial = data.get('local_trial', False)
    if type(trial) is not bool:
        return {'error': '试跑参数无效'}, 400
    owner = request.headers.get('X-Follow-Page', '')
    seq = data.get('seq')
    with motion_lock:
        if target is None:
            with lock:
                start_generation += 1
                if len(owner) == 36 and type(seq) is int:
                    page_actions[owner] = max(seq, page_actions.get(owner, 0))
                session.stop(time.monotonic())
            try:
                actuator_request('POST', '/stop', {})
            except (OSError, HTTPException, ValueError, RuntimeError):
                return {'error': '会话已取消，底盘停止确认暂未收到'}, 503
            return session.snapshot()
        with lock:
            now = time.monotonic()
            if VIEW_ONLY or shutting_down:
                return {'error': '当前不能启动跟随'}, 409
            if len(owner) != 36 or type(seq) is not int:
                return {'error': '请刷新控制页面'}, 400
            key = (owner, seq)
            cached = request_cache.get(key)
            if cached is not None:
                if (cached['generation'] == start_generation and cached['payload'] == data
                        and cached['session_id'] == follow['session_id'] and session.armed):
                    return session.snapshot()
                return {'error': '此启动请求已取消或参数已改变'}, 409
            if seq <= page_actions.get(owner, 0):
                return {'error': '启动请求已过期'}, 409
            snapshot = data.get('start_status')
            if not (isinstance(snapshot, list) and len(snapshot) == 3
                    and snapshot[0] == START_SESSION and snapshot[2] == start_generation
                    and type(snapshot[1]) in (int, float) and now - 2.5 <= snapshot[1] <= now):
                return {'error': '页面状态已更新，请等待刷新后重试'}, 409
            if session.armed:
                return {'error': '请先停止当前跟随'}, 409
            if not state['enabled'] or not control_link['ready']:
                return {'error': '请等待识别和底盘就绪'}, 503
            if target not in AUTO_TARGETS and target not in state['ids']:
                return {'error': '所选人物编号已失效，请重新选择'}, 409
            page_actions[owner] = seq
        try:
            actuator_request('POST', '/arm', {})
        except (OSError, HTTPException, ValueError, RuntimeError) as exc:
            return {'error': str(exc)}, 503
        with lock:
            session.start(target, time.monotonic(), LOCAL_TRIAL_SECONDS if trial else None)
            request_cache[key] = dict(generation=start_generation, payload=data, session_id=follow['session_id'])
            while len(request_cache) > 256:
                del request_cache[next(iter(request_cache))]
            return session.snapshot()


@app.post('/shutdown')
def shutdown_host():
    global start_generation, shutting_down
    data = request.get_json(silent=True) or {}
    if not isinstance(data, dict) or data.get('confirm') != 'shutdown-host' or data.get('boot_id') != START_SESSION:
        return {'error': '请确认关闭车载主机'}, 400
    if not EDGE_MODEL or VIEW_ONLY:
        return {'error': '仅车端控制入口可关闭主机'}, 409
    with motion_lock:
        with lock:
            shutting_down = True
            start_generation += 1
            session.stop(time.monotonic(), '准备关闭车载主机')
        try:
            actuator_request('POST', '/stop', {})
            deadline = time.monotonic() + 2
            stopped_since = None
            while time.monotonic() < deadline:
                health = actuator_request('GET', '/health')
                sent, odom = health.get('sent', {}), health.get('odometry')
                stopped = (not health.get('active') and sent.get('forward_mps') == 0
                           and sent.get('turn_rad_s') == 0 and odom and odom['age_s'] < .5
                           and abs(odom['forward_mps']) < .01 and abs(odom['turn_rad_s']) < .02)
                if stopped:
                    stopped_since = stopped_since or time.monotonic()
                    if time.monotonic() - stopped_since >= .3:
                        break
                else:
                    stopped_since = None
                time.sleep(.05)
            else:
                raise RuntimeError('未确认底盘持续零速度，未执行关机')
            subprocess.run(['sudo', '-n', 'systemctl', 'poweroff'], check=True, timeout=3)
        except (OSError, HTTPException, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
            with lock:
                shutting_down = False
            return {'error': str(exc)}, 503
    return {'ok': True, 'message': '车载主机正在关闭，电池总开关仍需手动关闭'}


@app.post('/heartbeat')
def heartbeat():
    global last_page_heartbeat
    with lock:
        now = time.monotonic()
        if EDGE_MODEL:
            owner = request.headers.get('X-Follow-Page', '')
            if len(owner) != 36:
                return {'error': '请刷新跟随页面'}, 400
            for page, sent_at in list(page_heartbeats.items()):
                if now - sent_at > 60:
                    del page_heartbeats[page]
            page_heartbeats[owner] = now
        else:
            last_page_heartbeat = now
    return {'ok': True}


@app.get('/healthz')
def healthz():
    with lock:
        now = time.monotonic()
        return {'backend': 'robot' if EDGE_MODEL else 'wsl', 'view_only': VIEW_ONLY,
                'version': VERSION, 'image_age_s': now - observation_time if observation_time else None,
                'detection_enabled': state['enabled'],
                'inference_age_s': now - performance[-1][0] if performance else None,
                'control_age_s': now - state.get('control_tick_at', 0),
                'idle': not session.armed, 'session_id': follow['session_id']}


@app.get('/status')
def status():
    with lock:
        now = time.monotonic()
        age = now - observation_time
        fresh = age <= FRAME_TIMEOUT
        sensors = observation_sensors if fresh else None
        distances = {str(key): round(obs.lidar_distance_m - (SETTINGS.camera_to_front_m or 0), 3)
                     for key, obs in observations.items() if fresh and obs.lidar_distance_m is not None}
        view = session.snapshot()
        view['local_trial_seconds'] = round(max(0, session.deadline - now), 1) if session.deadline else None
        measurements = [item for item in performance if now - item[0] <= 30]
        latencies = sorted(item[1] * 1000 for item in control_performance if now - item[0] <= 30)
        fps = ((len(measurements) - 1) / max(.001, measurements[-1][0] - measurements[0][0])) if len(measurements) > 1 else 0
        return jsonify(status=state['status'], enabled=state['enabled'],
                       ids=sorted(state['ids']) if age <= DISPLAY_RESULT_TIMEOUT else [], follow=view,
                       distances_m=distances, sensors={'aligned': bool(sensors and sensors.has_depth),
                           'lidar': fresh_lidar(sensors, now) is not None,
                           'image_age_s': age if observation_time else None,
                           'lidar_age_s': now - sensors.lidar.at if sensors and sensors.lidar else None,
                           'trial_block_reason': None},
                       control_ready=control_link['ready'], control_error=control_link['error'],
                       battery_voltage=control_link.get('battery_voltage'), battery=control_link.get('battery', {}),
                       actuator={'sent': control_link.get('sent'), 'odometry': health_view(now).get('odometry')},
                       video=state['frame'] is not None, view_only=VIEW_ONLY,
                       backend='robot' if EDGE_MODEL else 'wsl', detect_ms=state['detect_ms'], version=VERSION,
                       performance={'fps': round(fps, 2), 'latency_p95_ms': round(latencies[min(len(latencies)-1, int(len(latencies)*.95))], 1) if latencies else None,
                                    'capture_fetch_ms': state.get('capture_fetch_ms'), 'capture_age_ms': state.get('capture_age_ms'),
                                    'worker_start_age_ms': state.get('worker_start_age_ms'), 'worker_end_age_ms': state.get('worker_end_age_ms'),
                                    'latency_basis': 'camera receive to control bridge acknowledgement'},
                       reid=state.get('reid', {}), calibration={'geometry': SETTINGS.geometry_verified,
                           'braking': SETTINGS.braking_verified, 'basis': '原厂模型，安装位置未改动'},
                       events=list(session.events)[-10:], start_status=[START_SESSION, now, start_generation])


@app.get('/video')
def video():
    def frames():
        previous = None
        while True:
            with lock:
                frame = state['frame']
            if frame is not None and frame is not previous:
                yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n'
                previous = frame
            time.sleep(0.04)
    return Response(frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.get('/preview.jpg')
def preview():
    with lock:
        frame = state['frame']
    return Response(frame, mimetype='image/jpeg') if frame is not None else ('', 503)


if __name__ == '__main__':
    threading.Thread(target=capture, daemon=True).start()
    threading.Thread(target=worker, daemon=True).start()
    threading.Thread(target=control_loop, daemon=True).start()
    app.run(host='127.0.0.1', port=VIEWER_PORT, threaded=True)
