"""Exercise ROS timer/watchdog on an isolated topic; never publishes to /cmd_vel."""
import json
from pathlib import Path
import sys
import time
import importlib.util
import hashlib
from types import SimpleNamespace
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist

sys.path.insert(0, '/tmp')
source = Path(sys.argv[1] if len(sys.argv) > 1 else '/tmp/follow_actuator.py')
spec = importlib.util.spec_from_file_location('watchdog_candidate', source)
candidate = importlib.util.module_from_spec(spec)
spec.loader.exec_module(candidate)
Actuator, COMMAND_TIMEOUT_S = candidate.Actuator, candidate.COMMAND_TIMEOUT_S

topic = '/codex_validation/cmd_vel'
rclpy.init(args=['--ros-args', '-r', '/cmd_vel:=' + topic, '-r', '__node:=codex_watchdog_validation'])
bridge = Actuator()
observer = Node('codex_watchdog_observer', use_global_arguments=False)
messages = []
observer.create_subscription(Twist, topic,
                             lambda message: messages.append((time.monotonic(), message.linear.x, message.angular.z)), 10)
executor = SingleThreadedExecutor()
executor.add_node(bridge)
executor.add_node(observer)


def spin_for(seconds):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        executor.spin_once(timeout_sec=.005)


try:
    assert bridge.publisher.topic_name == topic, 'Refusing: publisher was not remapped away from the chassis'
    spin_for(1.)
    assert bridge.publisher.get_subscription_count() == 1, 'Unexpected topic subscribers'
    elapsed = []
    for trial in range(10):
        bridge.battery.samples.clear()
        bridge.battery.low_since = bridge.battery.recovery_since = None
        bridge.battery.latched = False
        bridge.on_battery_voltage(SimpleNamespace(data=11.4))
        messages.clear()
        started = time.monotonic()
        bridge.accept({'active': True, 'forward_mps': .07, 'turn_rad_s': .1})
        spin_for(.42)
        moving = [sample for sample in messages if sample[1] > 0]
        assert moving, 'Test topic did not observe nonzero test data'
        zeros = [at for at, v, w in messages if at > moving[-1][0] and v == 0 and w == 0]
        assert zeros and not bridge.active
        assert bridge.stop_reason == '指令看门狗超时'
        elapsed.append((zeros[0] - started) * 1000)
        spin_for(.013 * (trial % 3))
    result = {'scope': 'ROS loop timing on isolated /codex_validation/cmd_vel; no chassis motion',
              'watchdog_s': COMMAND_TIMEOUT_S, 'trials': len(elapsed), 'zero_observed_ms': elapsed,
              'actuator_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
              'max_zero_observed_ms': max(elapsed), 'passed_350ms': max(elapsed) <= 350}
    Path('/tmp/wheeltec-watchdog-validation.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    assert result['passed_350ms']
finally:
    bridge.stop()
    spin_for(.1)
    executor.shutdown()
    observer.destroy_node()
    bridge.destroy_node()
    rclpy.shutdown()
