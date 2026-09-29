import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from swarm_interfaces.msg import RobotPose, RobotPoseArray, RobotGoal, RobotGoalArray, ProximitySensor
import traceback
from geometry_msgs.msg import Twist
import math
 
class RobotController(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("robot_controller") # MODIFY NAME

        ## Step 1: Get the name of the robot 
        self.declare_parameter('robot_name', '')
        self.robot_name = self.get_parameter('robot_name').value
        self.get_logger().info(f'{self.robot_name} robot controller has been started.')

        ## Step 2: Subscribe to /robot_pose_list
        self.pose_subscriber = self.create_subscription(RobotPoseArray, '/robot_pose_list', self.get_pose_callback, 10)
        self.robot_pose = RobotPose()

        ## Step 3: Subscribe to /robot_goal_list
        self.goal_subscriber = self.create_subscription(RobotGoalArray, '/robot_goal_list', self.get_goal_callback, 10)
        self.robot_goal = RobotGoal()
        self.goal_received = False

        ## Step 4: Subscribe to /{robot_name}/proximity_sensor_vals
        self.prox_sensor_subscriber = self.create_subscription(ProximitySensor, f'/{self.robot_name}/proximity_sensor_vals', self.get_prox_sensor_callback, 10)
        self.proximity_sensor_vals = ProximitySensor()


        ## Step 4: Initialise cmd_vel publisher
        self.cmd_vel = Twist()
        self.cmd_vel_publisher = self.create_publisher(Twist, f'/{self.robot_name}/cmd_vel', 10)
        self.create_timer(0.1, self.publish_cmd_vel)

        ## Controller Gains
        self.kp_linear = 0.3
        self.kp_angular = 0.45
        self.distance_threshold = 0.07

        ## Avoidance hysteresis state
        self.avoiding = False
        self.avoid_clear_ticks_needed = 2   # ~0.5s at 10Hz before trusting "clear"
        self.avoid_clear_counter = 0

    def get_pose_callback(self, msg:RobotPoseArray):
        pose_array = msg
        for element in pose_array.robot_pose:
            if element.robot_name == self.robot_name:
                self.robot_pose = element

        # self.get_logger().info(f"Robot Pose obtained: {self.robot_pose}")

    def get_goal_callback(self, msg:RobotGoalArray):
        goal_array = msg
        for element in goal_array.robot_goal:
            if element.robot_name == self.robot_name:
                self.robot_goal = element

        self.goal_received = True
        self.get_logger().info(f"Robot Goal obtained: {self.robot_goal}")

    def get_prox_sensor_callback(self, msg:ProximitySensor):
        self.proximity_sensor_vals = msg

    def calculate_go_to_goal_vel(self):
        distance_error = round(math.sqrt((self.robot_goal.x - self.robot_pose.x)**2 + (self.robot_goal.y - self.robot_pose.y)**2), 4)
        angle_to_goal = math.atan2(self.robot_goal.y - self.robot_pose.y, self.robot_goal.x - self.robot_pose.x)
        pose_theta_radians = math.radians(self.robot_pose.theta)
        heading_error = angle_to_goal - pose_theta_radians
        heading_error_normalised = round(math.atan2(math.sin(heading_error), math.cos(heading_error)),2)

        # self.get_logger().info(f"Angle to goal: {angle_to_goal}")
        self.get_logger().info(f"NHE:{heading_error_normalised}, Distance Error: {distance_error}")

        if distance_error < self.distance_threshold:
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0

        else:
            forward_component = max(0.0, math.cos(heading_error_normalised))
            self.cmd_vel.linear.x = self.kp_linear*distance_error*forward_component
            self.cmd_vel.angular.z = self.kp_angular*heading_error_normalised

    def calculate_avoid_obstacle_vel(self):
        distance_error = round(math.sqrt((self.robot_goal.x - self.robot_pose.x)**2 + (self.robot_goal.y - self.robot_pose.y)**2), 4)
        if distance_error < self.distance_threshold:
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0

        p = self.proximity_sensor_vals
        left, front, right = p.left_detected, p.front_detected, p.right_detected

        angle_to_goal = math.atan2(self.robot_goal.y - self.robot_pose.y, self.robot_goal.x - self.robot_pose.x)
        heading_error = math.atan2(math.sin(angle_to_goal - math.radians(self.robot_pose.theta)),
                                   math.cos(angle_to_goal - math.radians(self.robot_pose.theta)))

        goal_prefers_left = heading_error > 0

        relevant_dists = [d for d, trig in
                            [(p.proximity_left, left), (p.proximity_front, front), (p.proximity_right, right)]
                            if trig and d > 0]
        min_dist = min(relevant_dists) if relevant_dists else 0.1
        turn_mag = min(2.0, max(0.8, 0.2 / max(min_dist, 0.05)))

        if front:
            # Blocked ahead - direction is ambiguous, defer to the goal.
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = turn_mag if goal_prefers_left else -turn_mag
        elif left and right:
            # Squeezed on both sides but clear ahead - also ambiguous, defer to goal.
            self.cmd_vel.linear.x = 0.05
            self.cmd_vel.angular.z = turn_mag if goal_prefers_left else -turn_mag
        elif left:
            # Obstacle on the left only - turning right is unambiguous.
            self.cmd_vel.linear.x = 0.05
            self.cmd_vel.angular.z = -turn_mag
        elif right:
            self.cmd_vel.linear.x = 0.05
            self.cmd_vel.angular.z = turn_mag



    def publish_cmd_vel(self):
        if self.goal_received == True:
            obstacle_present = (self.proximity_sensor_vals.left_detected
                                or self.proximity_sensor_vals.front_detected
                                or self.proximity_sensor_vals.right_detected)

            if obstacle_present:
                self.avoiding = True
                self.avoid_clear_counter = 0
            elif self.avoiding:
                self.avoid_clear_counter += 1
                if self.avoid_clear_counter >= self.avoid_clear_ticks_needed:
                    self.avoiding = False

            if self.avoiding:
                self.calculate_avoid_obstacle_vel()
            else:
                self.calculate_go_to_goal_vel()

        self.cmd_vel_publisher.publish(self.cmd_vel)


    

 
def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None

    try:
        node = RobotController()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("Shutting down robot controller via keyboard interrupt.")
            pass
    except Exception:
        exit_code = 1
        if node is not None:
            node.get_logger().error(f"Robot Controller crashed:\n{traceback.format_exc()}")
        else:
            print(f"Robot Controller crashed during construction:\n{traceback.format_exc()}", flush=True)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()