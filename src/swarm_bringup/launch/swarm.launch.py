import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('swarm_bringup'), 'config', 'robot_config2.yaml')

    with open(config_path) as f:
        config = yaml.safe_load(f)

    robot_names = sorted(config['simulation_manager']['ros__parameters']['robots'].keys())
    print("Robot Names" + str(robot_names))

    simulation_manager_node = Node(
        package='swarm_coppelia_drivers',
        executable='simulation_manager',
        parameters=[config_path])

    return LaunchDescription([simulation_manager_node])