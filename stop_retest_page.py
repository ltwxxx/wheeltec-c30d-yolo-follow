"""Local, single-use page for recording a WheelTec stop test."""

import json
import logging
from pathlib import Path
import secrets
import shlex
import subprocess
import sys
import tempfile
import threading
import time
from urllib.request import urlopen

from flask import Flask, Response, jsonify, render_template_string, request


app = Flask(__name__)
app.config["TRUSTED_HOSTS"] = ["127.0.0.1"]
logging.getLogger("werkzeug").setLevel(logging.ERROR)
condition = threading.Condition()
state = {"phase": "ready"}
revision = 0
token = secrets.token_urlsafe(24)
expires = time.monotonic() + 15 * 60
follow_four = "--follow-four" in sys.argv
follow_pulse = follow_four or "--follow-pulse" in sys.argv
stop_requested = threading.Event()
last_heartbeat = 0.0
ssh = [
    "ssh", "-S", "/private/tmp/wheeltec-ssh.UOnouU/control-pdcn",
    "-o", "ControlMaster=no", "-o", "BatchMode=yes", "-o", "ConnectTimeout=4",
    "-o", "HostKeyAlias=192.168.0.100",
    "-o", "UserKnownHostsFile=/private/tmp/wheeltec-ssh.UOnouU/known_hosts",
    "-o", "StrictHostKeyChecking=yes", "wheeltec@192.168.1.242",
    "timeout -k 3 20 python3 -u -",
]


def publish(event):
    global state, revision
    with condition:
        state = event
        revision += 1
        condition.notify_all()


@app.after_request
def local_page_headers(response):
    response.headers["Content-Security-Policy"] = "frame-ancestors 'none'"
    response.headers["Cache-Control"] = "no-store"
    return response


def current_plan(expected_target=None):
    try:
        with urlopen("http://127.0.0.1:8765/status", timeout=0.35) as response:
            status = json.load(response)
    except (OSError, ValueError) as exc:
        raise ValueError("未能读取摄像头跟随联调状态") from exc
    follow = status["follow"]
    target = follow["target"]
    if target is None or not status["enabled"] or target not in status["ids"]:
        raise ValueError("请先在摄像头页选中一个人物并启动跟随联调")
    if expected_target is not None and expected_target != target:
        raise ValueError("跟随目标已变化，本次试转取消")
    if not status["sensors"]["aligned"] or follow["distance_m"] is None:
        raise ValueError("目标深度暂不可用，本次试转取消")
    x = round(follow["forward_mps"] * 1000)
    yaw = round(follow["turn_rad_s"] * 1000)
    if abs(yaw) >= 30:
        return target, {"forward": 0, "yaw": max(-150, min(150, yaw))}
    if x >= 20:
        return target, {"forward": min(70, x), "yaw": 0}
    raise ValueError("当前跟随计划没有可用于试转的速度")


def run_retest(expected_target=None):
    events = []
    cancel_reason = None
    try:
        code = Path(__file__).with_name("stop_retest_robot.py").read_text()
        command = ssh
        if follow_pulse:
            command = ssh[:-1] + [shlex.join(
                ["timeout", "-k", "3", "20", "python3", "-u", "-c", code,
                 "--follow-four" if follow_four else "--follow-pulse"]
            )]
        with subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True) as process:
            if not follow_pulse:
                process.stdin.write(code)
                process.stdin.close()
            for line in process.stdout:
                received = time.monotonic()
                event = json.loads(line)
                if event["phase"] in ("await_plan", "need_plan"):
                    try:
                        if follow_four and (stop_requested.is_set() or time.monotonic() - last_heartbeat > 0.75):
                            raise ValueError("网页已停止或不再处于前台")
                        _, plan = current_plan(expected_target)
                        if event["phase"] == "need_plan":
                            plan["seq"] = event["seq"]
                        process.stdin.write(json.dumps(plan) + "\n")
                        process.stdin.flush()
                    except (OSError, ValueError) as exc:
                        cancel_reason = str(exc)
                        if not process.stdin.closed:
                            process.stdin.close()
                    if not follow_four and not process.stdin.closed:
                        process.stdin.close()
                    if event["phase"] == "need_plan":
                        continue
                if event["phase"] in ("aborted", "result") and cancel_reason:
                    event["error"] = cancel_reason
                events.append({"mac_received": received, **event})
                publish({key: value for key, value in event.items() if key != "samples"})
        if not events or events[-1]["phase"] not in ("result", "aborted"):
            publish({"phase": "aborted", "error": "未收到完整测试结果，请目视确认车轮状态"})
    except (OSError, ValueError) as exc:
        publish({"phase": "aborted", "error": str(exc)})
    finally:
        if events:
            with tempfile.NamedTemporaryFile(mode="w", prefix="wheeltec-stop-", suffix=".json",
                                             dir="/private/tmp", delete=False) as log:
                json.dump(events, log)
                print("STOP_RETEST_LOG", log.name, flush=True)
        print("STOP_RETEST_STATUS", json.dumps(state, ensure_ascii=False), flush=True)


@app.get("/")
def home():
    return render_template_string("""<!doctype html>
<html lang="zh"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>{{ '小车悬空跟随试转' if follow_pulse else '小车停车复测' }}</title>
<style>
body{font-family:system-ui,sans-serif;background:#101923;color:#f5f7fa;margin:3vw auto;padding:0 1rem;max-width:56rem}
h1{font-size:clamp(1.5rem,4vw,3rem)} #cue{display:grid;place-items:center;min-height:33vh;border-radius:1rem;
font-size:clamp(1.7rem,5.5vw,5rem);font-weight:800;text-align:center;padding:.5rem;background:#283a53}
#cue[data-phase=drive]{background:#206447} #cue[data-phase=cutoff]{background:#a12c2c}
#cue[data-phase=zero]{background:#e2b241;color:#17202b}
#cue[data-phase=result]{background:#1d5f78}
button{background:#e9f2ff;border:0;padding:.8rem 1.2rem;font:inherit;font-weight:700;border-radius:.5rem;cursor:pointer}
button:disabled{opacity:.5;cursor:default} #details{font-size:1.1rem}
</style>
<h1>{{ '小车悬空跟随试转' if follow_pulse else '小车停车复测' }}</h1>
<div id="cue" aria-live="assertive">连接中</div>
<p id="details" aria-live="polite"></p>
{% if follow_four %}
<p>四轮稳固悬空，驱动电源开关在手边。先在<a href="http://127.0.0.1:8765/" target="_blank" style="color:#b8dbff">摄像头页</a>选中人物并启动跟随联调。</p>
<button id="start" disabled>开始最多 4 秒试转</button>
<button id="stop" disabled>停止试转</button>
<p>本次只用相机深度判断人物距离，约 1.4 米内停止；雷达不参与试转。请保持本页在前台。深度中断、目标丢失或计时结束时车端会发送零速。出现异常就立即切断驱动电源。</p>
{% elif follow_pulse %}
<p>四轮稳固悬空，驱动电源开关在手边。先在<a href="http://127.0.0.1:8765/" style="color:#b8dbff">摄像头页</a>选中人物并启动跟随联调。</p>
<button id="start" disabled>开始单次试转</button>
<p>试转按当前计划只发送一次不超过 0.4 秒的低速动作，随后立即补发零速。出现异常就切断驱动电源。</p>
{% else %}
<p>四轮悬空，慢动作录像同时拍到车轮和屏幕，驱动电源开关在手边。准备好后开始，观察红色提示出现前后车轮何时停转。</p>
<button id="start" disabled>开始单次复测</button>
<p>出现异常就立即切断驱动电源。屏幕提示经过 Wi-Fi，录像中读出的时间是近似值。</p>
{% endif %}
<script>
const cue = document.getElementById('cue');
const details = document.getElementById('details');
const start = document.getElementById('start');
const labels = {
  ready:'等待开始', connecting:'正在连接小车', countdown:'准备 · 约 3 秒', await_plan:'正在核对跟随计划', stopping:'正在停止',
  guard_brake:'车端保护进程正在发送零速',
  drive:'低速指令', cutoff:'已断指令', zero:'补发零速', result:'测试结束', aborted:'测试中止'
};
const events = new EventSource('/events');
events.onmessage = ({data}) => {
  const event = JSON.parse(data);
  cue.dataset.phase = event.phase;
  cue.textContent = labels[event.phase];
  start.disabled = event.phase !== 'ready';
  if (event.phase === 'result') {
    const stop = event.test === 'follow_pulse' || event.test === 'follow_four'
      ? ((event.test === 'follow_four' ? '运动指令阶段约 ' + (event.duration_ms/1000).toFixed(2) + ' 秒。' : '') +
         (event.saw_motion ? '底盘反馈检测到运动' : '底盘反馈未检测到明显运动'))
      : event.auto_stop_ms === null
      ? '观察窗口内未读到持续零速反馈'
      : '断指令约 ' + event.auto_stop_ms + ' ms 后读到持续零速反馈';
    details.textContent = (event.error ? event.error + '。' : '') + stop +
      '。补发零速后' + (event.zero_stable ? '读到持续零速反馈。' : '未确认零速反馈，请检查车轮。');
  } else {
    details.textContent = event.error || '';
  }
};
{% if follow_four %}
const stopButton = document.getElementById('stop');
setInterval(() => {
  if (document.visibilityState === 'visible') fetch('/heartbeat', {method:'POST',
    headers:{'Content-Type':'application/json'}, body:JSON.stringify({token:{{ token|tojson }}})}).catch(()=>{});
}, 200);
stopButton.onclick = () => {
  stopButton.disabled = true;
  fetch('/stop', {method:'POST', headers:{'Content-Type':'application/json'},
    body:JSON.stringify({token:{{ token|tojson }}})}).catch(()=>{});
};
const originalMessage = events.onmessage;
events.onmessage = (message) => {
  originalMessage(message);
  const phase = JSON.parse(message.data).phase;
  stopButton.disabled = !['connecting','await_plan','drive'].includes(phase);
};
{% endif %}
events.onerror = () => {
  start.disabled = true;
  details.textContent = '复测页连接中断，请观察车轮。';
};
start.onclick = async () => {
  start.disabled = true;
  try {
    const response = await fetch('/start', {method:'POST', headers:{'Content-Type':'application/json'},
      body:JSON.stringify({token:{{ token|tojson }}})});
    if (!response.ok) {
      details.textContent = (await response.json()).error;
      if (response.status === 422) start.disabled = false;
    }
  } catch (error) {
    details.textContent = '启动状态未知，请观察车轮和屏幕。';
  }
};
</script></html>""", token=token, follow_pulse=follow_pulse, follow_four=follow_four)


@app.post("/start")
def start_test():
    global last_heartbeat
    data = request.get_json(silent=True)
    supplied = data.get("token") if isinstance(data, dict) else None
    if not isinstance(supplied, str) or not supplied.isascii() or not secrets.compare_digest(supplied, token):
        return jsonify(error="无法验证本机复测请求"), 403
    expected_target = None
    if follow_pulse:
        try:
            expected_target, _ = current_plan()
        except ValueError as exc:
            return jsonify(error=str(exc)), 422
    with condition:
        if state["phase"] != "ready" or time.monotonic() >= expires:
            return jsonify(error="本次复测已触发或已过期"), 409
        last_heartbeat = time.monotonic()
        publish({"phase": "connecting"})
    threading.Thread(target=run_retest, args=(expected_target,), daemon=True).start()
    return jsonify(ok=True)


def authenticated():
    data = request.get_json(silent=True)
    supplied = data.get("token") if isinstance(data, dict) else None
    return isinstance(supplied, str) and supplied.isascii() and secrets.compare_digest(supplied, token)


@app.post("/heartbeat")
def heartbeat():
    global last_heartbeat
    if not authenticated():
        return jsonify(error="验证失败"), 403
    with condition:
        last_heartbeat = time.monotonic()
    return jsonify(ok=True)


@app.post("/stop")
def stop():
    if not authenticated():
        return jsonify(error="验证失败"), 403
    stop_requested.set()
    with condition:
        if state["phase"] not in ("ready", "zero", "result", "aborted", "guard_brake"):
            publish({"phase": "stopping"})
    return jsonify(ok=True)


@app.get("/events")
def stream():
    def updates():
        seen = -1
        while True:
            with condition:
                changed = condition.wait_for(lambda: seen != revision, timeout=10)
                seen = revision
                message = json.dumps(state)
            yield "data: " + message + "\n\n" if changed else ": keep-alive\n\n"

    return Response(updates(), mimetype="text/event-stream",
                    headers={"Cache-Control": "no-store", "X-Accel-Buffering": "no"})


if __name__ == "__main__":
    app.run(host="127.0.0.1", port=8768 if follow_four else 8767 if follow_pulse else 8766, threaded=True)
