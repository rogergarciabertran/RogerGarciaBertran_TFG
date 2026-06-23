from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'crawler_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    (os.path.join('share', package_name, 'config'), glob('config/*.json')),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='crawler',
    maintainer_email='crawler@todo.todo',
    description='Crawler control package (DAC, FG, motor, telemetry, joystick)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        "power_relay_node = crawler_control.nodes.power_relay_node:main",
        "camera_ip_node = crawler_control.nodes.camera_ip_node:main",
        "youtube_camera_node = crawler_control.nodes.youtube_camera_node:main",
        "telemetry_node = crawler_control.nodes.telemetry_node:main",
        "imu_node = crawler_control.nodes.imu_node:main",
        "dac_dual_node = crawler_control.nodes.dac_dual_node:main",
        "fg_dual_node = crawler_control.nodes.fg_dual_node:main",
        "joy_diff_node = crawler_control.nodes.joy_diff_node:main",
        "dual_fisheye_node = crawler_control.nodes.dual_fisheye_node:main",
        "udp_joy_node = crawler_control.nodes.udp_joy_node:main", 
   ],
},
)
