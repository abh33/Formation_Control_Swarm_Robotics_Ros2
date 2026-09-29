import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('swarm_bringup'), 'config', 'robot_config5.yaml')

    with open(config_path) as f:
        config = yaml.safe_load(f)

    yaml_data = config['simulation_manager']['ros__parameters']['robots']
    robot_names = sorted(yaml_data.keys())
    robot_lookup = {}
    for robot_name in robot_names:
        name = yaml_data[robot_name]['name']
        aruco_id = yaml_data[robot_name]['aruco_id']
        robot_lookup[f"{aruco_id}"] = name
    print(robot_lookup)

    simulation_manager_node = Node(
        package='swarm_coppelia_drivers',
        executable='simulation_manager',
        parameters=[config_path])

    camera_vision_sensor_node = Node(
        package='swarm_coppelia_drivers',
        executable='camera_vision_sensor_node',
        parameters=[{'robot_lookup': robot_lookup}])

    goal_allocation_node = Node(
        package='swarm_control',
        executable='goal_allocation',
        parameters=[{'robot_lookup': robot_lookup}])

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

    robot_controller_nodes = []
    for temp in robot_names:
        name = yaml_data[temp]['name']
        node = Node(
            package='swarm_control',
            executable='robot_controller',
            namespace=name,
            name='robot_controller',
            parameters=[{'robot_name': name}])
        robot_controller_nodes.append(node)

    return LaunchDescription([simulation_manager_node, *robot_driver_nodes, camera_vision_sensor_node, goal_allocation_node, *robot_controller_nodes])