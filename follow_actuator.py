"""Loopback-only ROS cmd_vel bridge with strict speed caps and a watchdog."""

import json
import math
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import signal
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


MAX_FORWARD_MPS = 0.08
MAX_TURN_RAD_S = 0.25
COMMAND_TIMEOUT_S = 0.30
STOP_BURST_S = 1.5


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
        self.create_timer(0.05, self.tick)

    def stop(self):
        with self.lock:
            self.active = False
            self.forward = 0.0
            self.turn = 0.0
            self.zero_until = time.monotonic() + STOP_BURST_S

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
            self.active = True
            self.forward = float(forward)
            self.turn = float(turn)
            self.last_command = time.monotonic()

    def tick(self):
        now = time.monotonic()
        with self.lock:
            if self.active and now - self.last_command <= COMMAND_TIMEOUT_S:
                forward, turn = self.forward, self.turn
            else:
                if self.active:
                    self.active = False
                    self.forward = 0.0
                    self.turn = 0.0
                    self.zero_until = now + STOP_BURST_S
                if now > self.zero_until:
                    return
                forward = turn = 0.0

        message = Twist()
        message.linear.x = forward
        message.angular.z = turn
        self.publisher.publish(message)

    def health(self):
        with self.lock:
            active = self.active
            age = time.monotonic() - self.last_command if active else None
        subscribers = self.publisher.get_subscription_count()
        return {"ok": subscribers > 0, "subscribers": subscribers,
                "active": active, "command_age_s": age,
                "max_forward_mps": MAX_FORWARD_MPS,
                "max_turn_rad_s": MAX_TURN_RAD_S}


def handler(actuator):
    class Request(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

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
