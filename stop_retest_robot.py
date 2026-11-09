"""Run one bounded stop test on the Raspberry Pi, with the wheels suspended."""

import fcntl
import functools
import json
import operator
import os
import select
import signal
import struct
import subprocess
import sys
import termios
import time
import tty


PORT = "/dev/wheeltec_controller"
SPEED_MM_S = 70
MAX_BENCH_YAW_MRAD_S = 150
COMMAND_INTERVAL = 0.05
COMMAND_COUNT = 8
OBSERVATION_SECONDS = 1.0
ZERO_SECONDS = 2.0
FOLLOW_SECONDS = 4.0
PLAN_TIMEOUT = 0.35


def mark(phase, **fields):
    record = {"phase": phase, "at": time.monotonic(), **fields}
    try:
        os.write(1, (json.dumps(record) + "\n").encode())
    except OSError:
        # Losing the display connection must not interrupt the local stop.
        pass


def checksum(data):
    return functools.reduce(operator.xor, data, 0)


def speed_packet(mm_s, yaw_mrad_s=0):
    data = struct.pack(">BBBhhh", 0x7B, 0, 0, mm_s, 0, yaw_mrad_s)
    return data + bytes((checksum(data), 0x7D))


class Controller:
    def __init__(self, fd):
        self.fd = fd
        self.buffer = bytearray()

    def send(self, speed, yaw=0):
        packet = speed_packet(speed, yaw)
        if os.write(self.fd, packet) != len(packet):
            raise OSError("short controller write")
        return time.monotonic()

    def collect(self, deadline, samples):
        while (remaining := deadline - time.monotonic()) > 0:
            readable, _, _ = select.select([self.fd], [], [], min(remaining, 0.025))
            if not readable:
                continue
            data = os.read(self.fd, 4096)
            received = time.monotonic()
            if not data:
                raise OSError("controller disconnected")
            self.buffer.extend(data)
            while len(self.buffer) >= 24:
                frame = self.buffer[:24]
                if (frame[0] == 0x7B and frame[23] == 0x7D
                        and checksum(frame[:22]) == frame[22]):
                    samples.append((received, *struct.unpack_from(">hhh", frame, 2)))
                    del self.buffer[:24]
                else:
                    del self.buffer[0]


def stationary(sample):
    return all(abs(speed) <= 4 for speed in sample[1:])


def stopped_since(samples):
    for index in range(len(samples) - 1, -1, -1):
        if not stationary(samples[index]):
            break
    else:
        index = -1
    tail = samples[index + 1:]
    if len(tail) >= 5 and tail[-1][0] - tail[0][0] >= 0.15:
        return tail[0][0]
    return None


def stop_with_zero(controller, samples, progress=None):
    for sig in (signal.SIGHUP, signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, signal.SIG_IGN)
    sent = 0
    mark("zero")
    deadline = time.monotonic() + ZERO_SECONDS
    while (now := time.monotonic()) < deadline:
        next_tick = min(now + COMMAND_INTERVAL, deadline)
        try:
            controller.send(0)
            sent += 1
            if progress is not None:
                try:
                    progress()
                except OSError:
                    pass
        except OSError:
            pass
        try:
            controller.collect(next_tick, samples)
        except OSError:
            time.sleep(max(0.0, next_tick - time.monotonic()))
    return sent


def require_stationary(controller, seconds, min_frames):
    baseline = []
    controller.collect(time.monotonic() + seconds, baseline)
    if len(baseline) < min_frames or not all(map(stationary, baseline)):
        raise RuntimeError("底盘反馈不足，或测试前未处于静止状态")
    if time.monotonic() - baseline[-1][0] > 0.15:
        raise RuntimeError("底盘静止反馈已超时")
    return baseline[-1]


def test(controller):
    mark("countdown", seconds=3)
    baseline = require_stationary(controller, 3.0, 20)

    during, silence, braking = [], [], []
    last_command = None
    error = None
    try:
        mark("drive")
        for _ in range(COMMAND_COUNT):
            recent = during[-1] if during else baseline
            if time.monotonic() - recent[0] > 0.2:
                raise RuntimeError("底盘反馈中断")
            last_command = controller.send(SPEED_MM_S)
            if _ < COMMAND_COUNT - 1:
                controller.collect(last_command + COMMAND_INTERVAL, during)
        mark("cutoff", last_command=last_command)
        controller.collect(last_command + OBSERVATION_SECONDS, silence)
    except (OSError, RuntimeError) as exc:
        error = str(exc)
    finally:
        # Always send zero even when the first motion write only partly succeeds.
        zero_sent = stop_with_zero(controller, braking)

    stopped = stopped_since(silence)
    mark("result",
         error=error,
         peak_x=max((abs(sample[1]) for sample in during + silence), default=0),
         silent_frames=len(silence),
         auto_stop_ms=round(1000 * (stopped - last_command)) if stopped is not None else None,
         zero_sent=zero_sent,
         zero_stable=stopped_since(braking) is not None,
         samples=[
             [phase, round(1000 * (at - last_command)), x, y, yaw]
             for phase, samples in (("drive", during), ("silent", silence), ("zero", braking))
             for at, x, y, yaw in samples
         ] if last_command is not None else [])


def read_pulse():
    deadline = time.monotonic() + 1.0
    data = bytearray()
    while b"\n" not in data and len(data) < 256:
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not select.select([0], [], [], remaining)[0]:
            raise RuntimeError("没有收到及时的跟随计划")
        chunk = os.read(0, 256 - len(data))
        if not chunk:
            raise RuntimeError("跟随计划已取消")
        data.extend(chunk)
    if b"\n" not in data:
        raise RuntimeError("试转指令不完整")
    try:
        plan = json.loads(data)
        forward, yaw = plan["forward"], plan["yaw"]
    except (ValueError, KeyError, TypeError) as exc:
        raise RuntimeError("无效的试转指令") from exc
    if (type(forward) is not int or type(yaw) is not int or not 0 <= forward <= SPEED_MM_S
            or not -MAX_BENCH_YAW_MRAD_S <= yaw <= MAX_BENCH_YAW_MRAD_S
            or bool(forward) == bool(yaw)):
        raise RuntimeError("试转只接受限幅后的单轴非零指令")
    return forward, yaw


def test_follow_pulse(controller):
    require_stationary(controller, 0.65, 8)
    mark("await_plan")
    forward, yaw = read_pulse()
    baseline = require_stationary(controller, 0.12, 2)
    during, braking = [], []
    error = None
    try:
        mark("drive", forward=forward, yaw=yaw)
        deadline = time.monotonic() + COMMAND_COUNT * COMMAND_INTERVAL
        for _ in range(COMMAND_COUNT):
            now = time.monotonic()
            if now >= deadline:
                break
            recent = during[-1] if during else baseline
            if now - recent[0] > 0.2:
                raise RuntimeError("底盘反馈中断")
            sent = controller.send(forward, yaw)
            controller.collect(min(deadline, sent + COMMAND_INTERVAL), during)
    except (OSError, RuntimeError) as exc:
        error = str(exc)
    finally:
        zero_sent = stop_with_zero(controller, braking)
    mark("result", test="follow_pulse", error=error, forward=forward, yaw=yaw,
         zero_sent=zero_sent, zero_stable=stopped_since(braking) is not None,
         saw_motion=any(not stationary(sample) for sample in during + braking),
         samples=during + braking)


def pulse_from_data(data):
    try:
        plan = json.loads(data)
        forward, yaw, seq = plan["forward"], plan["yaw"], plan["seq"]
    except (ValueError, KeyError, TypeError) as exc:
        raise RuntimeError("无效的跟随计划") from exc
    if (type(forward) is not int or type(yaw) is not int or type(seq) is not int
            or not 0 <= forward <= SPEED_MM_S or not -MAX_BENCH_YAW_MRAD_S <= yaw <= MAX_BENCH_YAW_MRAD_S
            or bool(forward) == bool(yaw)):
        raise RuntimeError("跟随计划超出单轴速度限制")
    return seq, forward, yaw


def guard_stop(fd, heartbeat, parent):
    """Independent local process takes over zero output if the writer stalls."""
    for sig in (signal.SIGHUP, signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, signal.SIG_IGN)
    os.close(0)
    started = last_heartbeat = time.monotonic()
    deadline = started + FOLLOW_SECONDS + 0.5
    braking = False
    while True:
        now = time.monotonic()
        if now - last_heartbeat > PLAN_TIMEOUT or now >= deadline:
            break
        readable, _, _ = select.select([heartbeat], [], [], min(0.05, deadline - now))
        if not readable:
            continue
        data = os.read(heartbeat, 1024)
        if not data:
            break
        for event in data:
            if event == ord("e"):
                os._exit(0)
            if event == ord("s") and not braking:
                braking = True
                deadline = time.monotonic() + ZERO_SECONDS + 0.5
            if event in (ord("m"), ord("s")):
                last_heartbeat = time.monotonic()

    if os.getppid() == parent:
        try:
            os.kill(parent, signal.SIGSTOP)
        except ProcessLookupError:
            pass
    mark("guard_brake", error="车端控制进程超时，独立进程正在发送零速")
    until = time.monotonic() + ZERO_SECONDS
    while time.monotonic() < until:
        try:
            os.write(fd, speed_packet(0))
        except OSError:
            pass
        time.sleep(COMMAND_INTERVAL)
    if os.getppid() == parent:
        try:
            os.kill(parent, signal.SIGKILL)
        except ProcessLookupError:
            pass
    os._exit(1)


def test_follow_four(controller):
    require_stationary(controller, 0.65, 8)
    mark("await_plan")
    forward, yaw = read_pulse()
    baseline = require_stationary(controller, 0.12, 2)
    read_fd, write_fd = os.pipe()
    try:
        guardian = os.fork()
    except OSError:
        os.close(read_fd)
        os.close(write_fd)
        raise
    if guardian == 0:
        os.close(write_fd)
        guard_stop(controller.fd, read_fd, os.getppid())
    os.close(read_fd)
    os.set_blocking(0, False)
    data = bytearray()
    during, braking = [], []
    error = None
    began = time.monotonic()
    updated = began
    next_request = began
    outstanding = None
    seq = 0
    actual = 0.0
    try:
        mark("drive", seconds=FOLLOW_SECONDS)
        while time.monotonic() - began < FOLLOW_SECONDS:
            now = time.monotonic()
            if now - updated > PLAN_TIMEOUT:
                raise RuntimeError("跟随数据超时")
            recent = during[-1] if during else baseline
            if now - recent[0] > 0.2:
                raise RuntimeError("底盘反馈中断")
            if outstanding is None and now >= next_request:
                seq += 1
                outstanding = seq
                mark("need_plan", seq=seq)
            if select.select([0], [], [], 0)[0]:
                chunk = os.read(0, 512)
                if not chunk:
                    raise RuntimeError("跟随连接已断开")
                data.extend(chunk)
                if len(data) > 1024:
                    raise RuntimeError("跟随数据格式错误")
                while b"\n" in data:
                    raw, _, remaining = data.partition(b"\n")
                    data = bytearray(remaining)
                    received_seq, forward, yaw = pulse_from_data(raw)
                    if received_seq != outstanding:
                        raise RuntimeError("跟随数据顺序错误")
                    outstanding = None
                    updated = time.monotonic()
                    next_request = updated + 0.1
            sent = controller.send(forward, yaw)
            os.write(write_fd, b"m")
            controller.collect(min(began + FOLLOW_SECONDS, sent + COMMAND_INTERVAL), during)
        actual = time.monotonic() - began
    except (OSError, RuntimeError) as exc:
        error = str(exc)
        actual = time.monotonic() - began
    finally:
        zero_sent = stop_with_zero(controller, braking, lambda: os.write(write_fd, b"s"))
        try:
            os.write(write_fd, b"e")
        except OSError:
            pass
        os.close(write_fd)
        os.waitpid(guardian, 0)
    mark("result", test="follow_four", error=error, duration_ms=round(actual * 1000),
         zero_sent=zero_sent, zero_stable=stopped_since(braking) is not None,
         saw_motion=any(not stationary(sample) for sample in during + braking),
         samples=during + braking)


def on_signal(signum, _frame):
    raise InterruptedError(f"测试中断（信号 {signum}）")


def main():
    if sys.argv[1:] not in ([], ["--follow-pulse"], ["--follow-four"]):
        raise RuntimeError("未知的测试模式")
    os.set_blocking(1, False)
    for sig in (signal.SIGHUP, signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, on_signal)
    busy = subprocess.run(["fuser", PORT], capture_output=True, timeout=2)
    if busy.returncode != 1:
        raise RuntimeError("底盘串口正在使用")
    fd = os.open(PORT, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    saved = termios.tcgetattr(fd)
    try:
        fcntl.ioctl(fd, termios.TIOCEXCL)
        tty.setraw(fd)
        settings = termios.tcgetattr(fd)
        settings[4] = settings[5] = termios.B115200
        termios.tcsetattr(fd, termios.TCSANOW, settings)
        termios.tcflush(fd, termios.TCIFLUSH)
        controller = Controller(fd)
        if sys.argv[1:] == ["--follow-four"]:
            test_follow_four(controller)
        elif sys.argv[1:]:
            test_follow_pulse(controller)
        else:
            test(controller)
    finally:
        termios.tcsetattr(fd, termios.TCSANOW, saved)
        os.close(fd)


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
        mark("aborted", error=str(exc))
        raise SystemExit(1)
