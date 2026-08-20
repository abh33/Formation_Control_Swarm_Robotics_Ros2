import rclpy
from rclpy.node import Node
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from swarm_interfaces.msg import ProximitySensor

class RobotDriver(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("robot_driver") # MODIFY NAME

        ## Step 1: Get the name of the robot 
        self.declare_parameter('robot_name', '')
        self.robot_name = self.get_parameter('robot_name').value
        self.get_logger().info(f'{self.robot_name} robot driver has been started.')

        # Step 2: Establish connection to CoppeliaSim
        self.connect_to_coppelia()

        # Step 3: Create a subscriber for /simulation_state
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.simulation_state = None
        self.simulation_state_subscriber = self.create_subscription(String, '/simulation_state', self.update_simulation_state, qos)

        # Step 4: Get all relevant robot handles
        self.robot_handle = None
        self.robot_left_motor_handle = None
        self.robot_right_motor_handle = None
        self.robot_proximity_sensor_left_handle = None
        self.robot_proximity_sensor_front_handle = None
        self.robot_proximity_sensor_right_handle = None
        self.initialise_robot_handles()

        # Step 5: Declare robot constants
        self.wheel_radius = 0.03204
        self.wheel_displacement = 0.1877


        # Step 6: Create a subscriber for /{robot_name}/cmd_vel topic
        self.cmd_vel_subscriber = self.create_subscription(Twist, f'/{self.robot_name}/cmd_vel', self.cmd_vel_callback, 10)

        ## Step 7: Create publisher for proximity sensor values
        self.prox_sensor_publisher = self.create_publisher(ProximitySensor, f'/{self.robot_name}/proximity_sensor_vals', 10)
        self.create_timer(0.2, self.publish_proximity_vals)


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

    def update_simulation_state(self, msg:String):
        """
        This is a callback function which is called everytime the simulation state topic publishes
        a mew message and updates the simulation state based on that.
        """
        self.simulation_state = msg.data

    def initialise_robot_handles(self):
        """
        This function initialises all the coppeliasim handles for robot, left and right motors
        and 3 proximity sensors
        """

        while rclpy.ok() and (self.simulation_state is None or self.simulation_state != "RUNNING"):
            rclpy.spin_once(self, timeout_sec=0.5)

        try:
            self.robot_handle = self.sim.getObject(f"/{self.robot_name}")
            self.robot_left_motor_handle = self.sim.getObject(f"/{self.robot_name}/left_joint")
            self.robot_right_motor_handle = self.sim.getObject(f"/{self.robot_name}/right_joint")
            self.robot_proximity_sensor_left_handle = self.sim.getObject(f"/{self.robot_name}/proximity_sensor_left")
            self.robot_proximity_sensor_front_handle = self.sim.getObject(f"/{self.robot_name}/proximity_sensor_front")
            self.robot_proximity_sensor_right_handle = self.sim.getObject(f"/{self.robot_name}/proximity_sensor_right")

            if self.robot_handle == -1:
                raise RuntimeError("Robot handle not valid")
            if self.robot_left_motor_handle == -1:
                raise RuntimeError("Robot left motor handle not valid")
            if self.robot_right_motor_handle == -1:
                raise RuntimeError("Robot right motor handle not valid")
            if self.robot_proximity_sensor_left_handle == -1:
                raise RuntimeError("Robot proximity left handle not valid")
            if self.robot_proximity_sensor_front_handle == -1:
                raise RuntimeError("Robot proximity front handle not valid")
            if self.robot_proximity_sensor_right_handle == -1:
                raise RuntimeError("Robot proximity right handle not valid")

            self.get_logger().info("All robot handles initialised correctly")
        except Exception as e:
            self.get_logger().error(f"Failed to get object handle: {str(e)}")
            raise e

    def cmd_vel_callback(self, msg:Twist):
        """
        This function receives the robot linear and angular velocity from cmd_vel topic and converts
        it into wheel velocities for left and right motor (in rad/s). Then the wheel velocities are
        written to the corresponding motors.
        """
        if self.simulation_state != "RUNNING":
            return

        linear_x = msg.linear.x
        angular_z = msg.angular.z

        v_left = (linear_x - (angular_z * self.wheel_displacement / 2)) / self.wheel_radius
        v_right = (linear_x + (angular_z * self.wheel_displacement / 2)) / self.wheel_radius

        self.sim.setJointTargetVelocity(self.robot_left_motor_handle, v_left)
        self.sim.setJointTargetVelocity(self.robot_right_motor_handle, v_right)

    def publish_proximity_vals(self):
        """
        This function reads the 3 proximity sensors associated with the robot and publishes the values
        to the topic /{robot_name}/proximity_sensor_vals
        """
        if self.simulation_state != "RUNNING":
            return

        detected_left , dist_left, _, _, _ = self.sim.readProximitySensor(self.robot_proximity_sensor_left_handle)
        detected_front , dist_front, _, _, _ = self.sim.readProximitySensor(self.robot_proximity_sensor_front_handle)
        detected_right , dist_right, _, _, _ = self.sim.readProximitySensor(self.robot_proximity_sensor_right_handle)

        dist_left = round(dist_left, 3)
        dist_front = round(dist_front, 3)
        dist_right = round(dist_right, 3)            
        # self.get_logger().info(f'{self.robot_name} : dist_left-{dist_left} dist_front-{dist_front} dist_right -{dist_right}')
        msg = ProximitySensor()
        msg.left_detected = detected_left
        msg.front_detected = detected_front
        msg.right_detected = detected_right
        msg.proximity_left = dist_left
        msg.proximity_front = dist_front
        msg.proximity_right = dist_right

        self.prox_sensor_publisher.publish(msg)
        

 
 
def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None

    try:
        node = RobotDriver()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("Shutting down robot driver via keyboard interrupt.")
            pass
    except Exception as e:
        print(f"Robot driver failed: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()