import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'intel_sys_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*') if os.path.exists('maps') else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aahil Shaikh',
    maintainer_email='aahil.aref@gmail.com',
    description='Odometry publisher, EKF sensor fusion, Point-LIO SLAM, and Nav2 stack configuration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_publisher = intel_sys_navigation.odom_publisher:main',
        ],
    },
)
