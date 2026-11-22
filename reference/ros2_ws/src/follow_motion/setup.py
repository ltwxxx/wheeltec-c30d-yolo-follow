from setuptools import setup

package_name = 'follow_motion'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    # 阶段 A 讲过的两坑之一：data_files 决定"文件去 install/ 哪儿"，
    # 第三个 ('share/' + package_name, []) 是包能被 ros2 pkg find 看到的根因。
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    # entry_points 决定 ros2 run <pkg> <exe> 里的 <exe> 是什么名字
    entry_points={
        'console_scripts': [
            'motion_node = follow_motion.motion_node:main',
        ],
    },
)
