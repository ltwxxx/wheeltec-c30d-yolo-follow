"""TF helpers run without ROS while preserving ROS message shape."""
import sys
import types
import numpy as np
import pytest


class Header:
    def __init__(self):
        self.frame_id = ''
        self.stamp = types.SimpleNamespace(sec=0, nanosec=0)


class Transform:
    def __init__(self):
        self.header = Header()
        self.child_frame_id = ''
        self.transform = types.SimpleNamespace(
            translation=types.SimpleNamespace(x=0., y=0., z=0.),
            rotation=types.SimpleNamespace(x=0., y=0., z=0., w=1.))


class Node:
    def __init__(self, name):
        self.parameters = {
            'color_image_topic': '/camera/color/image_raw',
            'depth_image_topic': '/camera/depth/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
            'scan_topic': '/scan',
            'http_port': 18767,
        }

    def declare_parameter(self, name, default):
        self.parameters[name] = default

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self.parameters[name])

    def create_subscription(self, *args):
        pass


@pytest.fixture
def sensors(monkeypatch):
    class QoS:
        pass
    modules = {
        'rclpy': {},
        'rclpy.node': {'Node': Node},
        'rclpy.qos': {'qos_profile_sensor_data': QoS()},
        'rclpy.time': {'Time': object},
        'rclpy.signals': {'SignalHandlerOptions': object},
        'sensor_msgs': {},
        'sensor_msgs.msg': {'CameraInfo': object, 'Image': object, 'LaserScan': object},
        'tf2_msgs': {},
        'tf2_msgs.msg': {'TFMessage': object},
    }
    for name, attributes in modules.items():
        module = types.ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
    opencv = types.ModuleType('cv2')
    opencv.setNumThreads = lambda _: None
    monkeypatch.setitem(sys.modules, 'cv2', opencv)
    import follow_bridges.follow_sensors as module
    return module


def transform(parent, child, x=0., y=0., yaw=0.):
    message = Transform()
    message.header.frame_id = parent
    message.child_frame_id = child
    message.transform.translation.x = x
    message.transform.translation.y = y
    message.transform.rotation.z = np.sin(yaw / 2)
    message.transform.rotation.w = np.cos(yaw / 2)
    return message


def translation(value):
    return np.array([value.translation.x, value.translation.y, value.translation.z])


def test_direct_inverse_and_chained_transforms(sensors):
    bridge = sensors.Sensors()
    bridge.on_transform(types.SimpleNamespace(transforms=[
        transform('odom', 'base_link', .1, .2)]))
    bridge.on_transform_static(types.SimpleNamespace(transforms=[
        transform('base_link', 'camera_frame', 1., 2., np.pi / 2),
        transform('base_link', 'laser_frame', .5, 0.),
    ]))

    direct = bridge.transform_between('base_link', 'camera_frame')
    assert np.allclose(translation(direct), (1., 2., 0.))
    projected = np.array([1., 0., 0.]) @ sensors.rotation_matrix(direct.rotation).T + translation(direct)
    assert np.allclose(projected, (1., 3., 0.), atol=1e-9)

    inverse = bridge.transform_between('camera_frame', 'base_link')
    assert np.allclose(translation(inverse), (-2., 1., 0.), atol=1e-9)

    chained = bridge.transform_between('camera_frame', 'laser_frame')
    chained_point = np.array([1., 0., 0.]) @ sensors.rotation_matrix(chained.rotation).T + translation(chained)
    direct_point = np.array([1., 0., 0.]) @ sensors.rotation_matrix(direct.rotation).T + translation(direct)
    inverse_point = direct_point @ sensors.rotation_matrix(inverse.rotation).T + translation(inverse)
    assert np.allclose(inverse_point, (1., 0., 0.), atol=1e-9)
    expected = (1., 0., 0.) @ sensors.rotation_matrix(inverse.rotation).T
    expected = expected + translation(inverse) + np.array([.5, 0., 0.]) @ sensors.rotation_matrix(inverse.rotation).T
    assert np.allclose(chained_point, expected, atol=1e-9)
    assert bridge.transform_between('camera_frame', 'camera_frame') is not None
    assert bridge.transform_between('missing', 'laser_frame') is None


def test_inverse_180_degree_rotation_is_normalized(sensors):
    value = transform('base_link', 'camera_frame', 1., 2., np.pi)
    value.transform.rotation.z = 1.
    value.transform.rotation.w = 0.
    with np.errstate(invalid='raise'):
        result = sensors.inverse_transform(value.transform)
    assert np.allclose(translation(result), (1., 2., 0.), atol=1e-9)
    assert np.allclose(sensors.rotation_matrix(result.rotation),
                       sensors.rotation_matrix(value.transform.rotation).T, atol=1e-9)


def test_invalid_transform_does_not_hide_other_paths(sensors):
    bridge = sensors.Sensors()
    invalid = transform('base_link', 'camera_frame')
    invalid.transform.rotation.w = 0.
    bridge.on_transform_static(types.SimpleNamespace(transforms=[invalid,
        transform('camera_frame', 'base_link', 1.)]))
    result = bridge.transform_between('base_link', 'camera_frame')
    assert np.allclose(translation(result), (-1., 0., 0.))


def test_invalid_direct_transform_does_not_hide_multihop_path(sensors):
    bridge = sensors.Sensors()
    invalid = transform('base_link', 'camera_frame')
    invalid.transform.rotation.w = 0.
    bridge.on_transform_static(types.SimpleNamespace(transforms=[invalid,
        transform('base_link', 'intermediate', 1.),
        transform('intermediate', 'camera_frame', 2.)]))
    result = bridge.transform_between('base_link', 'camera_frame')
    assert np.allclose(translation(result), (3., 0., 0.))


def test_invalid_inverse_transform_does_not_hide_other_paths(sensors):
    bridge = sensors.Sensors()
    invalid = transform('camera_frame', 'base_link')
    invalid.transform.rotation.w = 0.
    bridge.on_transform_static(types.SimpleNamespace(transforms=[invalid,
        transform('base_link', 'intermediate', 1.),
        transform('intermediate', 'camera_frame', 2.)]))
    result = bridge.transform_between('base_link', 'camera_frame')
    assert np.allclose(translation(result), (3., 0., 0.))
