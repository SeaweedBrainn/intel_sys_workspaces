from setuptools import setup
import os
from glob import glob

package_name = 'intel_sys_localization'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aahil Shaikh',
    maintainer_email='aahil.aref@gmail.com',
    description='State estimation, odometry integration, and sensor fusion for Intel Sys robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_publisher = intel_sys_localization.odom_publisher:main',
        ],
    },
)
