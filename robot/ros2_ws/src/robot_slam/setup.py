from setuptools import setup
from glob import glob
import os

package_name = 'robot_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cynthiawei1209',
    maintainer_email='cynthiawei1209@todo.todo',
    description='Robot SLAM package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'odom_node = robot_slam.odom_node:main',
        ],
    },
)