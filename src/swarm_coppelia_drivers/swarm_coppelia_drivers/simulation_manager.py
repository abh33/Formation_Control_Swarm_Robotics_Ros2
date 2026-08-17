import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
from rclpy.signals import SignalHandlerOptions


class SimulationManager(Node):
    def __init__(self):
        """Initializes the ROS 2 node and sets up the CoppeliaSim environment."""
        super().__init__("simulation_manager", automatically_declare_parameters_from_overrides=True)
        self.get_logger().info("Simulation Manager has started!")


        # Step 1: Resolve the absolute path to the .ttt scene file
        self.scene_path = self.get_scene_file_path()

        # Step 2: Establish connection to CoppeliaSim
        self.connect_to_coppelia()

        # Step 3: Check status and safely prepare the target scene
        self.initialize_simulation_scene()

        ## Step 4: Verify scene - Arena and overhead camera should be part of scene
        self.verify_flag = self.verify_simulation_scene()
        if not self.verify_flag:
            raise RuntimeError("Scene verification failed — missing /arena or /arena/overheadCamera")
        self.get_logger().info("Scene is OK")

        ## Step 5: Spawn Robots
        self.spawn_robots()

        ## Step 6: Start Simulation
        self.sim.startSimulation()

        ## Step 7: Publish simulation state to topic

        # This line defines the qos profile with buffer depth=1 with durability policy TRANSIENT_LOCAL
        # which means the publisher will "save" the last sent message. If a new subscriber connects after 
        # the message was already sent, the publisher will automatically re-send that last message to them.
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # Create a publisher to publish the simulation state
        self.state_pub = self.create_publisher(String, '/simulation_state', qos)
        # define last state
        self._last_state = None
        # Create timer to be called every 0.5 seconds
        self.create_timer(0.5, self._monitor_callback)  # 2 Hz

    

    def _monitor_callback(self):
        """
        This is a callback function called 2 times per second. This callback function
        checks the simulation state using the Coppeliasim api and publishes the simulation
        state to a topic named /simulation_state
        """            
        try:
            raw = self.sim.getSimulationState()
            state = self._map_state(raw)
        except Exception:
            state = 'ERROR'

        if state != self._last_state:
            self.get_logger().info(f'Sim state: {self._last_state} -> {state}')
            self._last_state = state

        self.state_pub.publish(String(data=state))

    def _map_state(self, raw):
        """
        Maps the raw output of sim.getSimulationState() to the 3 values - STOPPED, 
        PAUSED and RUNNING.
        Returns:
        Simulation state - STOPPED, PAUSED and RUNNING
        """
        # CoppeliaSim's sim.getSimulationState() returns an integer:
        # sim.simulation_stopped = 0
        # sim.simulation_paused = 8
        # sim.simulation_advancing_running = 17 (and other "advancing_*" codes while running)
        if raw == 0:
            return 'STOPPED'
        elif raw == 8:
            return 'PAUSED'
        else:
            return 'RUNNING'

    def stop_simulation(self):
        """
        Stops the Coppeliasim Simulation.
        """
        try:
            self.sim.stopSimulation()
            self.get_logger().info('Simulation stopped.')
        except Exception as e:
            self.get_logger().warn(f'Failed to stop simulation cleanly: {e}')

    def get_scene_file_path(self) -> str:
        """
        Locates the package share directory and constructs the target scene path.
        
        Returns:
            str: The full absolute system path to the .ttt file.
        """
        package_share_dir = get_package_share_directory('swarm_coppelia_drivers')
        scene_path = os.path.join(package_share_dir, 'scenes', 'swarm_ros2.ttt')
        
        self.get_logger().info(f'Located CoppeliaSim scene file at: {scene_path}')
        return scene_path

    def get_model_file_path(self) -> str:
        """
        Locates the package share directory and constructs the target model path.
        
        Returns:
            str: The full absolute system path to the .ttm file.
        """
        package_share_dir = get_package_share_directory('swarm_coppelia_drivers')
        model_path = os.path.join(package_share_dir, 'models', 'swarm_robot_model.ttm')
        
        self.get_logger().info(f'Located CoppeliaSim scene file at: {model_path}')
        return model_path

    def get_image_file_path(self, filename) -> str:
        """
        Locates the package share directory and constructs the target image path.
        
        Returns:
            str: The full absolute system path to the .png file.
        """
        package_share_dir = get_package_share_directory('swarm_coppelia_drivers')
        image_path = os.path.join(package_share_dir, 'markers', 'png', filename)
        
        self.get_logger().info(f'Located CoppeliaSim scene file at: {image_path}')
        return image_path

    def connect_to_coppelia(self):
        """
        Initializes the ZeroMQ Remote API clients to establish communication 
        with the running CoppeliaSim application instance.
        """
        try:
            self.coppelia_client = RemoteAPIClient()
            self.sim = self.coppelia_client.require('sim')
            self.get_logger().info("Successfully connected to CoppeliaSim Remote API Client.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to CoppeliaSim: {str(e)}")
            raise e


    def initialize_simulation_scene(self):
        """
        Manages scene loading lifecycle logic.
        """
        self.get_logger().info("Loading target scene from scratch...")
        self.sim.loadScene(self.scene_path)

    def verify_simulation_scene(self) -> bool:
        """
        Verify if the active scene contains the required 'arena' 
        and relative 'overheadCamera' hierarchy.
        """
        try:
            # Fetch handles using absolute and relative paths
            self.arena_handle = self.sim.getObject("/arena")
            self.camera_handle = self.sim.getObject("/arena/overheadCamera")

            # In ZeroMQ API, checking for handle validity (depends on API configuration)
            if self.arena_handle == -1 or self.camera_handle == -1:
                self.get_logger().warn("Scene verification failed: Missing required objects.")
                return False

            return True

        except Exception as e:
            # Capturing 'e' gives you the exact reason (e.g., connection drop, bad path syntax)
            self.get_logger().error(f"Exception during scene verification: {e}")
            return False

    def extract_yaml_data(self) -> None:
        """
        Reads the robot configuration data from the robot config YAML file,
        parsing nested dictionary structures deterministically.
        """
        # 1. Fetch raw flattened parameters from ROS 2
        raw_params = self.get_parameters_by_prefix('robots')
        
        # 2. Parse the flattened dotted parameter names into a structured dict
        parsed_robots = {}
        for dotted_name, param in raw_params.items():
            parts = dotted_name.split('.')
            
            # Guard against malformed or shallow parameters
            if len(parts) < 2:
                continue
                
            robot_id, field = parts[0], parts[1]
            parsed_robots.setdefault(robot_id, {})[field] = param.value

        # 3. Build a sorted list of dictionaries for deterministic spawn ordering
        self.robots = []
        for robot_id in sorted(parsed_robots.keys()):
            fields = parsed_robots[robot_id]
            
            # Inject the key identifier (e.g., 'robot_0') as 'id' to prevent 
            # overriding the YAML parameter name field ('swarm_robot_0')
            self.robots.append({
                'id': robot_id,
                **fields
            })

    def spawn_robots(self) -> None:
        """
        Extracts robot configuration data and handles the sequential spawning loop.
        """
        self.extract_yaml_data()

        if not self.robots:
            self.get_logger().warn("No robots found to spawn. Check configuration files.")
            return

        for robot_info in self.robots:
            # Uses explicit string keys matching your YAML layout
            name = robot_info['name']
            aruco_id = int(robot_info['aruco_id'])
            pos_x = float(robot_info['x'])
            pos_y = float(robot_info['y'])

            self.spawn_robot(name, aruco_id, pos_x, pos_y)


    def spawn_robot(self, name: str, aruco_id: int, x: float, y: float) -> None:
        """
        Spawns an individual robot model, updates its alias, dynamic texture, and position.
        """
        # 1. Resolve paths using local variables to prevent state overwriting
        model_path = self.get_model_file_path()
        robot_handle = self.sim.loadModel(model_path)
        
        self.get_logger().info(f"Spawning '{name}' (ID: {aruco_id}) at [{x}, {y}]")

        # 2. Update object identity
        self.sim.setObjectAlias(robot_handle, name)
        
        # 3. Handle relative path resolution reliably using modern CoppeliaSim format
        # Note: If name is dynamic, using the base handle approach prevents root-path failures
        aruco_handle = self.sim.getObject(f"./aruco_marker", {"proxy": robot_handle})
        if aruco_handle == -1:
            # Fallback to structural absolute path look-up if relative search fails
            aruco_handle = self.sim.getObject(f"/{name}/aruco_marker")

        # 4. Generate and Apply Aruco Texture
        aruco_image_name = f"5x5_1000-{aruco_id}.png"
        image_path = self.get_image_file_path(aruco_image_name)
        
        # Use explicit modern parameter mapping documentation flags
        texture_options = 0  
        shape_handle, texture_id, _ = self.sim.createTexture(image_path, texture_options)
        
        uv_scaling = [0.125, 0.125]
        self.sim.setShapeTexture(aruco_handle, texture_id, self.sim.texturemap_plane, -1, uv_scaling)
        self.sim.removeObjects([shape_handle]) # Clean up temporary generation shape container

        # 5. Coordinate Translation & Placement
        current_pos = self.sim.getObjectPosition(robot_handle, self.sim.handle_world)
        pos_z = current_pos[2] # Maintain target model default ground clearance / Z offset
        
        self.sim.setObjectPosition(robot_handle, [x, y, pos_z], self.sim.handle_world)


def main(args=None):
    """Main execution block to spin the ROS 2 node lifecycle."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    
    try:
        node = SimulationManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("Shutting down Simulation Manager via keyboard interrupt.")
            pass
    except Exception as e:
        print(f"Simulation Manager failed: {e}")
    finally:
        if node is not None:
            node.stop_simulation()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
