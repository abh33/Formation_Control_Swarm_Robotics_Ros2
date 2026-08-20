import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'swarm_coppelia_drivers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # ADD THIS LINE: Include your CoppeliaSim scene files
        (os.path.join('share', package_name, 'scenes'), glob(os.path.join('scenes', '*.ttt'))),

        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*.ttm'))),

        (os.path.join('share', package_name, 'markers', 'png'), glob(os.path.join('markers', 'png', '*.png'))),

        (os.path.join('share', package_name, 'markers', 'svg'), glob(os.path.join('markers', 'svg', '*.svg'))),

        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abh33',
    maintainer_email='abh33@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "hello_node = swarm_coppelia_drivers.hello_node:main",
            "simulation_manager = swarm_coppelia_drivers.simulation_manager:main",
            "robot_driver = swarm_coppelia_drivers.robot_driver:main",
            "camera_vision_sensor_node = swarm_coppelia_drivers.camera_vision_sensor_node:main"
        ],
    },
)
