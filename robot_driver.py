import rclpy
from rclpy.node import Node
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class RobotDriver(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("robot_driver") # MODIFY NAME
        self.get_logger().info("Robot Driver has been started.")

        self.coppelia_client = RemoteAPIClient()
        self.sim = self.coppelia_client.require('sim')

        self.loop_timer = self.create_timer(1.0, self.control_loop)
        self.loop_timer.cancel()
        self.loop_counter = 0

    def start_simulation(self):
        simulation_state = self.sim.getSimulationState()
        self.get_logger().info(str(simulation_state))
        if simulation_state == 0:
            self.sim.startSimulation()
        else:
            self.sim.stopSimulation()
            self.sim.startSimulation()

    def stop_simulation(self):
        simulation_state = self.sim.getSimulationState()
        self.get_logger().info(str(simulation_state))
        if simulation_state != 0:
            self.sim.stopSimulation()

    def control_loop(self):
        self.get_logger().info("Hello")
        self.loop_counter = self.loop_counter+1
        if self.loop_counter > 5:
            self.loop_timer.cancel()
            self.stop_simulation()
 
 
def main(args=None):
    rclpy.init(args=args)
    node = RobotDriver() # MODIFY NAME
    node.start_simulation()
    node.loop_timer.reset()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()