from setuptools import setup

package_name = 'follow_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/follow_bringup.launch.py']),
        ('share/' + package_name + '/config', ['config/follow_bridge_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='bruan',
    maintainer_email='root@todo.todo',
    description='Launch files and ROS 2 parameters for the Wheeltec follow application.',
    license='Apache-2.0',
)
