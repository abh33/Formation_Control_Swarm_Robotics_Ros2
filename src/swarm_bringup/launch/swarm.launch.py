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

    yaml_data = config['simulation_manager']['ros__parameters']['robots']
    robot_names = sorted(yaml_data.keys())
    print("Robot Names" + str(robot_names))
    print(str(yaml_data))


    simulation_manager_node = Node(
        package='swarm_coppelia_drivers',
        executable='simulation_manager',
        parameters=[config_path])

    robot_driver_nodes = []
    for temp in robot_names:
        name = yaml_data[temp]['name']
        node = Node(
            package='swarm_coppelia_drivers',
            executable='robot_driver',
            namespace=name,
            name='robot_driver',
            parameters=[{'robot_name': name}])
        robot_driver_nodes.append(node)

    return LaunchDescription([simulation_manager_node, *robot_driver_nodes])