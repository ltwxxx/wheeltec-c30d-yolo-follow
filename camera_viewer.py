from http.client import HTTPConnection, HTTPException
import json
import logging
import os
import threading
import time

import cv2
from flask import Flask, Response, jsonify, request
import torch
from ultralytics import YOLO

from follow_control import FRAME_TIMEOUT, fetch_frame, observe_person, plan_follow

app = Flask(__name__)
app.config["TRUSTED_HOSTS"] = ["127.0.0.1", "localhost"]
logging.getLogger("werkzeug").setLevel(logging.ERROR)
DISPLAY_RESULT_TIMEOUT = 1.25
lock = threading.Lock()
state = {'enabled': False, 'status': '正在连接相机', 'frame': None, 'ids': [], 'generation': 0}
follow = {'target': None, 'offset': None, 'distance_m': None, 'forward_mps': 0.0,
          'turn_rad_s': 0.0, 'status': '跟随联调已关闭'}
observations = {}
observation_time = 0.0
observation_sensors = None
WEIGHTS = os.environ.get('WHEELTEC_YOLO_WEIGHTS', '/private/tmp/wheeltec-yolo11n.pt')
DEVICE = 'mps' if torch.backends.mps.is_available() else 'cpu'
torch.set_num_threads(2)
torch.set_num_interop_threads(1)
cv2.setNumThreads(1)
latest = threading.Condition()
raw = None
sequence = 0
capture_epoch = 0
SENSOR_ERRORS = (OSError, HTTPException, ValueError, KeyError, TypeError, cv2.error)
CONTROL_URL = "127.0.0.1"
CONTROL_PORT = 18768
CONTROL_TIMEOUT = 0.22
PAGE_HEARTBEAT_TIMEOUT = 1.25
_control_http = None
_control_http_lock = threading.Lock()
control_link = {'ready': False, 'subscribers': 0, 'error': '正在连接底盘控制桥'}
last_page_heartbeat = 0.0


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
    was_active = False
    next_health = 0.0
    while True:
        now = time.monotonic()
        if now >= next_health:
            try:
                health = actuator_request('GET', '/health')
                with lock:
                    control_link.update(ready=bool(health.get('ok')),
                                        subscribers=int(health.get('subscribers', 0)),
                                        error='' if health.get('ok') else '底盘 /cmd_vel 订阅端未就绪')
            except (OSError, HTTPException, ValueError, RuntimeError) as exc:
                with lock:
                    control_link.update(ready=False, subscribers=0,
                                        error=f'底盘控制桥不可用：{exc}')
            next_health = now + 0.5

        with lock:
            if follow['target'] is not None:
                if not state['enabled']:
                    stop_follow('已停止：识别关闭')
                elif now - last_page_heartbeat > PAGE_HEARTBEAT_TIMEOUT:
                    stop_follow('已停止：摄像头网页已离开前台')
                elif not control_link['ready']:
                    stop_follow('已停止：底盘控制通道不可用')
                else:
                    update_follow(now)
            active = follow['target'] is not None and control_link['ready']
            forward = follow['forward_mps'] if active else 0.0
            turn = follow['turn_rad_s'] if active else 0.0

        if active:
            try:
                publish_actuator_command(True, forward, turn)
                was_active = True
            except (OSError, HTTPException, ValueError, RuntimeError) as exc:
                with lock:
                    control_link.update(ready=False, subscribers=0,
                                        error=f'底盘控制通道中断：{exc}')
                    stop_follow('已停止：底盘控制通道中断')
                was_active = False
        elif was_active:
            try:
                publish_actuator_command(False)
            except (OSError, HTTPException, ValueError, RuntimeError):
                pass
            was_active = False
        time.sleep(0.1)


def stop_follow(reason, distance=None):
    follow.update(target=None, offset=None, distance_m=distance,
                  forward_mps=0.0, turn_rad_s=0.0, status=reason)


def update_follow(now):
    target = follow['target']
    if target is None:
        return
    if now - observation_time > FRAME_TIMEOUT:
        stop_follow('已停止：画面或深度超时')
        return
    if target not in observations:
        stop_follow('已停止：所选人物编号丢失')
        return
    observation = observations[target]
    plan = plan_follow(observation, observation_sensors, None, now, require_lidar=False)
    distance = round(observation.distance_m, 2) if observation.distance_m is not None else None
    if plan.blocked:
        stop_follow(f'已停止：{plan.reason}', distance)
        return
    follow.update(offset=round(observation.offset, 2), distance_m=distance,
                  forward_mps=round(plan.forward_mps, 3), turn_rad_s=round(plan.turn_rad_s, 3),
                  status=f'前进跟随 · 人物 {target} · {plan.reason}')


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
            if follow['target'] is not None:
                stop_follow(reason)


def capture():
    last_stamp = None
    misses = 0
    failure_status = '相机数据桥连接失败（检查小车相机节点和 SSH 转发）'
    while True:
        tick = time.monotonic()
        try:
            stamp, image, captured_at, sensors = fetch_frame()
            if stamp == last_stamp:
                misses += 1
                failure_status = '相机画面没有更新，正在重连'
            else:
                publish_frame(image, captured_at, sensors)
                last_stamp = stamp
                misses = 0
        except SENSOR_ERRORS:
            misses += 1
            failure_status = '相机数据桥连接失败（检查小车相机节点和 SSH 转发）'
        if misses >= 3:
            disconnect(failure_status)
            last_stamp = None
            misses = 0
        time.sleep(max(0.03, 0.1 - (time.monotonic() - tick)))


def worker():
    global observation_time, observation_sensors
    model = None
    seen = 0
    previous_generation = None
    while True:
        with latest:
            latest.wait_for(lambda: raw is not None and sequence != seen)
            (frame, frame_time, sensors), seen, epoch = raw, sequence, capture_epoch
        with lock:
            enabled, generation = state['enabled'], state['generation']
        ids = []
        tracked_people = {}
        status = '识别已关闭'
        failed = False
        if enabled:
            try:
                if model is None:
                    with lock:
                        state['status'] = '正在加载模型'
                    model = YOLO(WEIGHTS)
                session = (generation, epoch)
                if session != previous_generation and model.predictor is not None:
                    for tracker in getattr(model.predictor, 'trackers', ()):
                        tracker.reset()
                previous_generation = session
                result = model.track(frame, persist=True, tracker='bytetrack.yaml', classes=[0],
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
                status = f'模型错误：{exc}'
                failed = True
        ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        with latest:
            if raw is None or epoch != capture_epoch:
                continue
            with lock:
                if generation != state['generation']:
                    continue
                state.update(status=status, ids=ids)
                observations.clear()
                observations.update(tracked_people)
                observation_time = frame_time
                observation_sensors = sensors
                update_follow(time.monotonic())
                if failed:
                    state['enabled'] = False
                if ok:
                    state['frame'] = jpeg.tobytes()


@app.get('/')
def home():
    return '''<!doctype html><html lang="zh"><meta charset="utf-8"><title>小车摄像头</title>
    <style>body{font-family:system-ui;background:#101923;color:#eef4f8;text-align:center;margin:2em}img{max-width:100%;border-radius:12px}button{font-size:18px;padding:12px 24px;margin:8px;border:0;border-radius:8px;cursor:pointer}</style>
    <h1>小车摄像头</h1><p id="status">连接中</p><img src="/video" alt="实时摄像头">
    <p><button onclick="setDetection(true)">启动人物识别与画面跟踪</button><button onclick="setDetection(false)">关闭识别</button></p>
    <p><select id="target" aria-label="人物编号"></select>
    <button id="start" onclick="setFollow(Number(document.getElementById('target').value))" disabled>启动前进跟随</button>
    <button onclick="setFollow(null)">停止跟随</button></p>
    <p id="follow-status" aria-live="polite">跟随联调已关闭</p>
    <p id="sensors"></p><p id="plan"></p>
    <p>限速 0.08 m/s；保持约 1.42 m 距离。画面、深度或控制连接失效时停车。开始前确认路线畅通，停止按钮可随时停车。</p><script>
    const target = document.getElementById('target');
    const start = document.getElementById('start');
    const followStatus = document.getElementById('follow-status');
    let pageState;
    async function post(path, data) {
      const response = await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});
      if (!response.ok) throw new Error((await response.json()).error);
    }
    async function setDetection(enabled){await post('/detection', {enabled}); await refresh();}
    async function setFollow(id){
      try { await post('/follow', {target:id}); await refresh(); }
      catch (error) { followStatus.textContent = error.message; }
    }
    async function heartbeat(){
      if (document.visibilityState !== 'visible') return;
      try { await fetch('/heartbeat',{method:'POST',cache:'no-store'}); } catch(e) {}
    }
    setInterval(heartbeat,400);heartbeat();
    function showPlan() {
      if (!pageState) return;
      const {follow, distances_m} = pageState;
      const meters = follow.target === null ? distances_m[target.value] : follow.distance_m;
      const distance = meters == null ? '未测得' : meters.toFixed(2) + ' m';
      document.getElementById('plan').textContent = '目标距离 ' + distance + ' · 计划前进 ' +
        follow.forward_mps.toFixed(3) + ' m/s · 计划转向 ' + follow.turn_rad_s.toFixed(3) + ' rad/s';
    }
    target.addEventListener('change', () => { showPlan(); updateStart(); });
    function updateStart() {
      start.disabled = !pageState || !pageState.ids.length || pageState.follow.target !== null ||
        !pageState.sensors.aligned || pageState.distances_m[target.value] == null ||
        !pageState.control_ready;
    }
    function updateTargets(s) {
      const signature = s.ids.map(id => id + ':' + (s.distances_m[String(id)] != null)).join(',');
      if (signature !== target.dataset.signature) {
        target.replaceChildren(...s.ids.map(id => new Option(
          '人物 ' + id + (s.distances_m[String(id)] == null ? ' · 深度待确认' : ' · 深度可用'), String(id))));
        target.dataset.signature = signature;
      }
      if (s.follow.target !== null) {
        if (s.ids.includes(s.follow.target)) target.value = String(s.follow.target);
        return;
      }
      const selected = Number(target.value);
      const selectedHasDepth = s.ids.includes(selected) && s.distances_m[String(selected)] != null;
      if (!selectedHasDepth) {
        const withDepth = s.ids.find(id => s.distances_m[String(id)] != null);
        if (withDepth != null) target.value = String(withDepth);
        else if (s.ids.includes(selected)) target.value = String(selected);
        else target.value = s.ids.length ? String(s.ids[0]) : '';
      }
    }
    async function refresh(){
      try {
        const r = await fetch('/status', {cache:'no-store'});
        const s = await r.json();
        pageState = s;
        document.getElementById('status').textContent = s.status;
        updateTargets(s);
        updateStart();
        followStatus.textContent = s.follow.status;
        const controlText = s.control_ready ? '底盘控制已连接' : '底盘控制未连接：' + s.control_error;
        document.getElementById('sensors').textContent = s.sensors.aligned
          ? '对齐深度可用 · ' + controlText
          : '对齐深度暂不可用 · ' + controlText;
        showPlan();
      } catch(e) {
        document.getElementById('status').textContent = '网页服务已断开';
        start.disabled = true;
      }
    }
    setInterval(refresh,500);refresh();</script></html>'''


@app.post('/detection')
def detection():
    data = request.get_json(silent=True) or {}
    if not isinstance(data, dict) or type(data.get('enabled')) is not bool:
        return {'error': 'invalid request'}, 400
    with lock:
        if state['enabled'] != data['enabled']:
            state.update(enabled=data['enabled'], ids=[], frame=None)
            state['generation'] += 1
            state['status'] = '正在启动识别' if data['enabled'] else '识别已关闭'
        if not data['enabled']:
            observations.clear()
            stop_follow('已停止：识别关闭')
    return {'ok': True}


@app.post('/follow')
def set_follow():
    data = request.get_json(silent=True)
    if not isinstance(data, dict) or 'target' not in data:
        return {'error': '请选择人物编号'}, 400
    target = data['target']
    stop_requested = target is None
    with lock:
        if target is None:
            stop_follow('前进跟随已停止')
        else:
            now = time.monotonic()
            if not control_link['ready']:
                return {'error': control_link['error'] or '底盘控制桥未就绪'}, 503
            if now - last_page_heartbeat > PAGE_HEARTBEAT_TIMEOUT:
                return {'error': '请保持摄像头网页处于前台后重试'}, 409
            if (type(target) is not int or not state['enabled']
                    or target not in observations or now - observation_time > FRAME_TIMEOUT):
                return {'error': '目标不可用，请等待识别后重试'}, 409
            follow['target'] = target
            update_follow(now)
            if follow['target'] is None:
                return {'error': follow['status']}, 409
        response = follow.copy()
    if stop_requested:
        try:
            actuator_request('POST', '/stop', {})
        except (OSError, HTTPException, ValueError, RuntimeError) as exc:
            with lock:
                control_link.update(ready=False, subscribers=0,
                                    error=f'停止指令未确认：{exc}')
    return jsonify(response)


@app.post('/heartbeat')
def heartbeat():
    global last_page_heartbeat
    with lock:
        last_page_heartbeat = time.monotonic()
    return {'ok': True}


@app.get('/status')
def status():
    with lock:
        now = time.monotonic()
        update_follow(now)
        age = now - observation_time
        fresh = age <= FRAME_TIMEOUT
        display_fresh = age <= DISPLAY_RESULT_TIMEOUT
        ids = sorted(state['ids']) if display_fresh else []
        sensors = observation_sensors if fresh else None
        distances = {str(track_id): round(obs.distance_m, 2) for track_id, obs in observations.items()
                     if fresh and obs.distance_m is not None}
        reported_follow = follow.copy()
        if reported_follow['target'] is None and reported_follow['status'] in (
                '已停止：画面或深度超时', '已停止：所选人物编号丢失',
                '已停止：目标丢失或画面超时', '已停止：底盘控制通道不可用',
                '已停止：底盘控制通道中断',
                '相机数据桥连接失败（检查小车相机节点和 SSH 转发）'):
            if ids and distances:
                reported_follow['status'] = '已停止：画面、深度和底盘连接已恢复；可选有深度的人后重新启动'
            elif fresh and state['enabled']:
                reported_follow['status'] = '已停止：等待稳定的人物编号和深度距离'
        display_status = state['status']
        if state['enabled'] and raw is None:
            display_status = state['status']
        elif state['enabled'] and observation_time and not fresh:
            display_status = ('识别结果稍旧，正在等待新画面' if display_fresh
                              else '识别结果超时，正在等待新画面')
        return jsonify(status=display_status, enabled=state['enabled'], ids=ids, follow=reported_follow,
                       distances_m=distances,
                       sensors={'aligned': sensors is not None},
                       control_ready=control_link['ready'], control_error=control_link['error'],
                       video=state['frame'] is not None)


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


if __name__ == '__main__':
    threading.Thread(target=capture, daemon=True).start()
    threading.Thread(target=worker, daemon=True).start()
    threading.Thread(target=control_loop, daemon=True).start()
    app.run(host='127.0.0.1', port=8765, threaded=True)
