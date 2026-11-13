"""Loopback-only ROS cmd_vel bridge with strict speed caps and a watchdog."""

import json
import math
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import signal
import threading
import time
import socket

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from battery_state import BatteryState
from follow_settings import SETTINGS, VERSION


LOW_FORWARD_MPS = 0.10
MAX_FORWARD_MPS = 0.25
FAST_MIN_VOLTAGE = 10.5
MAX_TURN_RAD_S = 0.25
COMMAND_TIMEOUT_S = 0.28
STOP_BURST_S = 1.5
BATTERY_SAMPLE_TIMEOUT_S = 5.0


class Actuator(Node):
    def __init__(self):
        super().__init__("codex_follow_actuator")
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.lock = threading.Lock()
        self.active = False
        self.forward = 0.0
        self.turn = 0.0
        self.last_command = 0.0
        self.zero_until = time.monotonic() + STOP_BURST_S
        self.battery_sample = None
        self.battery = BatteryState()
        self.odometry = None
        self.sent = {"forward_mps": 0.0, "turn_rad_s": 0.0, "at": 0.0}
        self.stop_reason = "启动待机"
        self.create_subscription(Float32, "/PowerVoltage", self.on_battery_voltage, 1)
        self.create_subscription(Odometry, "/odom", self.on_odometry, 1)
        self.create_timer(0.05, self.tick)

    def on_battery_voltage(self, message):
        voltage = float(message.data)
        with self.lock:
            self.battery_sample = ((time.monotonic(), voltage)
                                   if math.isfinite(voltage) and voltage > 0 else None)
            self.battery.observe(voltage, time.monotonic())

    def on_odometry(self, message):
        v, w = float(message.twist.twist.linear.x), float(message.twist.twist.angular.z)
        if math.isfinite(v) and math.isfinite(w):
            with self.lock:
                self.odometry = (time.monotonic(), v, w)

    def _publish(self, forward, turn, now):
        # Called under the same lock as stop/accept: no stale nonzero publish after stop.
        message = Twist()
        message.linear.x, message.angular.z = forward, turn
        self.publisher.publish(message)
        self.sent = {"forward_mps": forward, "turn_rad_s": turn, "at": now}

    def stop(self):
        with self.lock:
            self.active = False
            self.forward = 0.0
            self.turn = 0.0
            self.zero_until = time.monotonic() + STOP_BURST_S
            self.stop_reason = "人工停止或控制器暂停"
            self._publish(0.0, 0.0, time.monotonic())

    def _available_forward_mps(self, now):
        return self.battery.status(now)["limit_mps"]

    def arm(self):
        with self.lock:
            if self.active:
                raise RuntimeError("请先停止当前跟随")
            if not self.battery.arm(time.monotonic()):
                raise RuntimeError(self.battery.status(time.monotonic())["reason"])

    def accept(self, payload):
        active = payload.get("active")
        if type(active) is not bool:
            raise ValueError("active must be a boolean")
        if not active:
            self.stop()
            return

        forward = payload.get("forward_mps")
        turn = payload.get("turn_rad_s")
        if (isinstance(forward, bool) or not isinstance(forward, (int, float))
                or not math.isfinite(forward) or not 0.0 <= forward <= MAX_FORWARD_MPS
                or isinstance(turn, bool) or not isinstance(turn, (int, float))
                or not math.isfinite(turn) or abs(turn) > MAX_TURN_RAD_S):
            raise ValueError("velocity is outside the allowed range")
        if self.publisher.get_subscription_count() < 1:
            raise RuntimeError("底盘 /cmd_vel 订阅端未就绪")
        with self.lock:
            now = time.monotonic()
            if self._available_forward_mps(now) <= 0:
                raise RuntimeError(self.battery.status(now)["reason"])
            self.active = True
            self.forward = min(float(forward), self._available_forward_mps(now))
            self.turn = float(turn)
            self.last_command = now

    def tick(self):
        now = time.monotonic()
        with self.lock:
            permitted = self._available_forward_mps(now) > 0
            if self.active and now - self.last_command <= COMMAND_TIMEOUT_S and permitted:
                self.forward = min(self.forward, self._available_forward_mps(now))
                forward, turn = self.forward, self.turn
            else:
                if self.active:
                    self.stop_reason = "指令看门狗超时" if permitted else self.battery.status(now)["reason"]
                    self.active = False
                    self.forward = 0.0
                    self.turn = 0.0
                    self.zero_until = now + STOP_BURST_S
                if now > self.zero_until:
                    return
                forward = turn = 0.0

            self._publish(forward, turn, now)

    def health(self):
        now = time.monotonic()
        with self.lock:
            active = self.active
            age = now - self.last_command if active else None
            battery = self.battery_sample
            available_forward_mps = self._available_forward_mps(now)
            battery_status = self.battery.status(now)
            sent = dict(self.sent)
            odom = self.odometry
            reason = self.stop_reason
        voltage = round(battery[1], 2) if battery and now - battery[0] <= BATTERY_SAMPLE_TIMEOUT_S else None
        subscribers = self.publisher.get_subscription_count()
        return {"ok": subscribers > 0 and odom is not None and now - odom[0] <= 1., "subscribers": subscribers,
                "active": active, "command_age_s": age,
                "battery_voltage": voltage,
                "max_forward_mps": MAX_FORWARD_MPS,
                "available_forward_mps": available_forward_mps,
                "max_turn_rad_s": MAX_TURN_RAD_S, "battery": battery_status,
                "version": VERSION, "sent": sent, "stop_reason": reason,
                "odometry": ({"age_s": now - odom[0], "forward_mps": odom[1], "turn_rad_s": odom[2]}
                             if odom else None)}


def handler(actuator):
    class Request(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def setup(self):
            super().setup()
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        def reply(self, data, status=200):
            body = json.dumps(data, allow_nan=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path != "/health":
                self.reply({"error": "not found"}, 404)
                return
            status = actuator.health()
            self.reply(status, 200 if status["ok"] else 503)

        def do_POST(self):
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 <= length <= 2048:
                    raise ValueError("invalid request size")
                payload = json.loads(self.rfile.read(length))
                if not isinstance(payload, dict):
                    raise ValueError("request must be an object")
                if self.path == "/command":
                    actuator.accept(payload)
                elif self.path == "/stop":
                    actuator.stop()
                elif self.path == "/arm":
                    actuator.arm()
                else:
                    self.reply({"error": "not found"}, 404)
                    return
                self.reply({"ok": True})
            except (ValueError, TypeError, json.JSONDecodeError) as exc:
                self.reply({"error": str(exc)}, 400)
            except RuntimeError as exc:
                self.reply({"error": str(exc)}, 503)

        def log_message(self, *_args):
            pass

    return Request


def main():
    rclpy.init()
    actuator = Actuator()
    threading.Thread(target=rclpy.spin, args=(actuator,), daemon=True).start()
    server = ThreadingHTTPServer(("127.0.0.1", 18768), handler(actuator))
    server.daemon_threads = True

    def request_shutdown(_signum, _frame):
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, request_shutdown)
    signal.signal(signal.SIGINT, request_shutdown)
    print("FOLLOW_ACTUATOR_READY", flush=True)
    try:
        server.serve_forever()
    finally:
        server.server_close()
        actuator.stop()
        deadline = time.monotonic() + STOP_BURST_S
        while time.monotonic() < deadline:
            actuator.tick()
            time.sleep(0.05)
        rclpy.shutdown()


if __name__ == "__main__":
    main()
