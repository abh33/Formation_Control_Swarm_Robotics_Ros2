from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'swarm_control'

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
    maintainer='abh33',
    maintainer_email='abhinavsarkar2012@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "goal_allocation = swarm_control.goal_allocation:main",
            "robot_controller = swarm_control.robot_controller:main"
        ],
    },
)
