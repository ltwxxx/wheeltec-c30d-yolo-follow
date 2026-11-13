"""ROS stub exercises real actuator logic without accessing a robot."""
import sys
import types
import pytest


@pytest.fixture
def actuator(monkeypatch):
    class Twist:
        def __init__(self):
            self.linear = types.SimpleNamespace(x=0.)
            self.angular = types.SimpleNamespace(z=0.)
    class Publisher:
        def __init__(self):
            self.messages = []
        def get_subscription_count(self):
            return 1
        def publish(self, message):
            self.messages.append((message.linear.x, message.angular.z))
    class Node:
        def __init__(self, name):
            pass
        def create_publisher(self, *args):
            return Publisher()
        def create_subscription(self, *args):
            pass
        def create_timer(self, *args):
            pass
    for package, names in {'rclpy': {}, 'rclpy.node': {'Node': Node},
                           'geometry_msgs': {}, 'geometry_msgs.msg': {'Twist': Twist},
                           'nav_msgs': {}, 'nav_msgs.msg': {'Odometry': object},
                           'std_msgs': {}, 'std_msgs.msg': {'Float32': object}}.items():
        module = types.ModuleType(package)
        module.__dict__.update(names)
        monkeypatch.setitem(sys.modules, package, module)
    sys.modules.pop('follow_actuator', None)
    import follow_actuator as module
    clock = [10.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    bridge = module.Actuator()
    bridge.on_battery_voltage(types.SimpleNamespace(data=11.4))
    bridge.odometry = (10., 0., 0.)
    return bridge, clock


def test_watchdog_zero_within_350ms_and_stop_publishes_synchronously(actuator):
    bridge, clock = actuator
    bridge.arm()
    bridge.accept({'active': True, 'forward_mps': .25, 'turn_rad_s': .2})
    bridge.tick()
    assert bridge.publisher.messages[-1] == (.25, .2)
    clock[0] = 10.301
    bridge.tick()
    assert bridge.publisher.messages[-1] == (0., 0.)
    assert not bridge.active
    bridge.accept({'active': True, 'forward_mps': .1, 'turn_rad_s': .1})
    bridge.stop()
    assert bridge.publisher.messages[-1] == (0., 0.)
    bridge.tick()
    assert bridge.publisher.messages[-1] == (0., 0.)


def test_voltage_latch_stops_rotation_as_well_as_translation(actuator):
    bridge, clock = actuator
    bridge.accept({'active': True, 'forward_mps': .1, 'turn_rad_s': .2})
    for i in range(50):
        clock[0] = 10. + i*.1
        bridge.on_battery_voltage(types.SimpleNamespace(data=10.3))
        if not bridge.battery.latched:
            bridge.accept({'active': True, 'forward_mps': .1, 'turn_rad_s': .2})
        bridge.tick()
    assert bridge.battery.latched
    with pytest.raises(RuntimeError):
        bridge.accept({'active': True, 'forward_mps': 0., 'turn_rad_s': .2})
    bridge.tick()
    assert bridge.publisher.messages[-1] == (0., 0.)


def test_stale_battery_or_odometry_cannot_report_ready_or_allow_motion(actuator):
    bridge, clock = actuator
    assert bridge.health()['ok']
    clock[0] = 16.
    assert not bridge.health()['ok']
    with pytest.raises(RuntimeError):
        bridge.accept({'active': True, 'forward_mps': .1, 'turn_rad_s': 0.})
