from setuptools import setup

package_name = 'follow_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    entry_points={
        'console_scripts': [
            'sim_node = follow_sim.kinematics_node:main',
        ],
    },
)
