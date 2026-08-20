import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'intel_sys_hardware'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'udev'), glob('udev/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aahil Shaikh',
    maintainer_email='aahil.aref@gmail.com',
    description='Hardware driver interfaces for STM32 microcontroller, LiDAR, and RealSense camera',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stm32_bridge = intel_sys_hardware.ros_robot_controller_node:main',
            'mecanum_controller = intel_sys_hardware.mecanum_controller:main',
        ],
    },
)
