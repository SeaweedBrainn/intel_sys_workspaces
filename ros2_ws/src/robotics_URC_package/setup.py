from setuptools import find_packages, setup

package_name = 'robotics_URC_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='odysseus',
    maintainer_email='odysseus@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tree_node = robotics_URC_package.trees.main_tree:main',
            'robot = robotics_URC_package.main:main',
            'testbench_teleop = robotics_URC_package.slam.keyboard_teleop:main'
        ],
    },
)
