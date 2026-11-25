import glob
from setuptools import setup
package_name = 'follow_bridges'

setup(
    name=package_name,
    version='2.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resources/config', [
            'config/follow_settings.json', 'config/calibration.json', 'config/botsort_reid.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='bruan',
    maintainer_email='root@todo.todo',
    description='ROS 2 packaging for the existing sensor and actuator bridges.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'actuator_bridge = follow_bridges.actuator_bridge:main',
            'sensor_bridge = follow_bridges.sensor_bridge:main',
        ],
    },
)
