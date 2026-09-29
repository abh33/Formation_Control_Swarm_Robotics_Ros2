import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
import traceback
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import signal
from ament_index_python.packages import get_package_share_directory
import os
from swarm_interfaces.msg import RobotPose, RobotPoseArray, RobotGoal, RobotGoalArray
from scipy.optimize import linear_sum_assignment
import math


class GoalAllocation(Node):
    """
    Interactive goal-allocation node for the swarm.

    Displays the live overhead camera feed in an OpenCV window and lets the
    user pick per-robot goal locations either by double-clicking N points
    (SELECT_N mode) or by dragging out a curve (SHAPE_DRAW mode, which then
    resamples N evenly-spaced points along that curve). Once a full set of
    goals is selected, robots are matched to goals via Hungarian assignment
    (minimizing total travel distance) and the result is published as a
    RobotGoalArray on /robot_goal_list for robot_controller to consume.
    """

    def __init__(self):
        super().__init__("goal_allocation", automatically_declare_parameters_from_overrides=True)
        self.get_logger().info("Goal Allocation Node has started!")

        ## Step 1: Load the robot lookup to get the mapping between aruco id and
        ## robot name, and derive how many robots we're allocating goals for.
        self.number_of_robots = 0
        self.robot_id_to_name = {}
        self.extract_lookup_data()

        # Step 2: Subscribe to /simulation_state so other logic can gate on
        # whether CoppeliaSim is actually running (transient-local QoS so we
        # pick up the latest state even if we start after it was published).
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.simulation_state = None
        self.simulation_state_subscriber = self.create_subscription(String, '/simulation_state', self.update_simulation_state, qos)

        # Step 3: Subscribe to the raw camera feed. Each incoming frame is
        # converted to OpenCV format and re-rendered with the current
        # selection overlay via image_callback -> render().
        self.bridge = CvBridge()
        self.raw_frame = None
        self.image_subscriber = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # Step 4: Create the display window and register the mouse callback
        # that drives goal selection in both modes.
        self.window_name = "OpenCV Image Window"
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_click_event)

        # Step 5: Goal-selection state. Two mutually exclusive modes:
        #   SELECT_N   - click N individual goal points directly.
        #   SHAPE_DRAW - drag out a curve; N points are resampled from it.
        self.selection_modes = ["SELECT_N", "SHAPE_DRAW"]
        self.selection_mode = self.selection_modes[1]
        self.goal_points = []        # current goal points, in pixel coordinates
        self.shapedraw_points = []   # raw points sampled while dragging a shape
        self.drawing = False         # True while the left mouse button is held during a drag

        ## Step 6: Load the homography matrix (and its inverse) used to convert
        ## between camera pixel coordinates and the robots' real-world frame.
        self.homography_matrix = None
        self.homography_matrix_inv = None
        self.get_homography_matrix()

        ## Step 7: Subscribe to live robot poses (world coordinates, keyed by
        ## ArUco id) and set up the state used to hold the current goal
        ## assignment once matching has run.
        self.pose_subscriber = self.create_subscription(RobotPoseArray, '/robot_pose_list', self.get_pose_callback, 10)
        self.robot_id_to_pose = {}       # aruco_id -> (x_world, y_world), refreshed every pose message
        self.robot_pose_to_id = {}       # reverse of the above, rebuilt alongside it each message
        self.goal_points_world = []      # current goal set, converted to world coordinates
        self.robot_id_to_goal = {}       # aruco_id -> assigned goal, in world coordinates (published)
        self.robot_id_to_goal_pixel = {} # aruco_id -> assigned goal, in pixel coordinates (for the debug overlay)

        ## Step 8: Publisher for the final per-robot goal assignment.
        self.goal_publisher = self.create_publisher(RobotGoalArray, '/robot_goal_list', 10)

    def extract_lookup_data(self) -> None:
        """
        Reads the robot_lookup parameter block (ArUco ID -> robot name mapping)
        passed in via the launch-time YAML config, and rebuilds it into a plain
        {aruco_id: robot_name} dict (int-keyed) for fast lookup when publishing
        goals and logging.
        """
        # 1. Fetch the flattened 'robot_lookup' dictionary parameters
        raw_lookup = self.get_parameters_by_prefix('robot_lookup')

        # 2. Reconstruct the clean dictionary, converting the id to int so it
        #    matches the int aruco_id used everywhere else in this node.
        for key, param in raw_lookup.items():
            # Remove the 'id_' prefix to get your clean integer-like string back
            clean_id = key.replace('id_', '')
            clean_id = int(clean_id)
            self.robot_id_to_name[clean_id] = param.value

        self.number_of_robots = len(self.robot_id_to_name)
        self.get_logger().info(f"Successfully loaded lookup: {self.robot_id_to_name}")
        self.get_logger().info(f"Number of robots: {self.number_of_robots}")

    def update_simulation_state(self, msg: String):
        """
        Subscription callback for /simulation_state. Stores the latest published
        state string (e.g. "RUNNING", "STOPPED") so other methods can gate their
        behavior on it without querying CoppeliaSim directly.
        """
        self.simulation_state = msg.data

    def image_callback(self, msg):
        """
        Subscription callback for /camera/image_raw. Converts the incoming ROS
        Image message to an OpenCV BGR frame and immediately re-renders the
        display window so the feed stays live as new frames arrive.
        """
        try:
            self.raw_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.render()
        except CvBridgeError as e:
            print(f"Conversion failed: {e}")

    def mouse_click_event(self, event, x, y, flags, param):
        """
        OpenCV mouse callback for the display window. Behavior depends on the
        active selection_mode:

        SELECT_N:
          - Double left-click: add a goal point at (x, y). Once the goal set
            is already full (== number_of_robots), the oldest point is
            evicted (FIFO) to make room for the new one.
          - Double middle-click: clear the current goal selection and any
            existing assignment/overlay state.

        SHAPE_DRAW:
          - Left button down: start a new drag, discarding any previously
            drawn curve.
          - Mouse move while dragging: append the current point to the curve.
          - Left button up: finish the drag and immediately resample N
            evenly-spaced goal points along the drawn curve.
          - Middle button down: clear the drawn curve and any existing
            goal selection/assignment/overlay state.
        """
        if self.selection_mode == "SELECT_N":
            if event == cv2.EVENT_LBUTTONDBLCLK:
                if len(self.goal_points) < self.number_of_robots:
                    self.goal_points.append((x, y))
                elif len(self.goal_points) == self.number_of_robots:
                    self.goal_points.pop(0)
                    self.goal_points.append((x, y))

                # Recompute world-frame goals (and trigger matching) once
                # every click; get_goal_points_world only actually acts once
                # the set is complete.
                self.get_goal_points_world()

            elif event == cv2.EVENT_MBUTTONDBLCLK:
                # Clear everything derived from the previous goal set so no
                # stale assignment lingers (in state or on the overlay).
                self.goal_points = []
                self.goal_points_world = []
                self.robot_id_to_goal = {}
                self.robot_id_to_goal_pixel = {}

        elif self.selection_mode == "SHAPE_DRAW":
            if event == cv2.EVENT_LBUTTONDOWN:
                self.drawing = True
                # Start each new drag from a clean curve, so a second draw
                # doesn't get appended onto a leftover one from before.
                self.shapedraw_points = []
            elif event == cv2.EVENT_MOUSEMOVE:
                if self.drawing == True:
                    self.shapedraw_points.append((x, y))
            elif event == cv2.EVENT_LBUTTONUP:  # true when left button released
                self.drawing = False
                # Curve is complete - resample it into goal points and match.
                self.get_goal_points_world()

            elif event == cv2.EVENT_MBUTTONDOWN:
                # Clear the drawn curve plus everything derived from it.
                self.shapedraw_points = []
                self.goal_points = []
                self.goal_points_world = []
                self.robot_id_to_goal = {}
                self.robot_id_to_goal_pixel = {}

    def render(self):
        """
        Redraws the debug overlay on top of the latest raw camera frame and
        shows it in the display window. Always starts from a fresh copy of
        raw_frame so nothing accumulates across frames; what gets drawn
        depends on the current selection_mode and the current assignment:

          - SELECT_N: red circles at each clicked goal point.
          - SHAPE_DRAW: yellow circles along the drawn curve, red circles at
            the resampled goal points.
          - Both modes: a blue line from each robot's current position to
            its currently assigned goal, for any robot that already has one.
        """
        if self.raw_frame is None:
            return
        display = self.raw_frame.copy()

        if self.selection_mode == "SELECT_N":
            for coord in self.goal_points:
                cv2.circle(display, coord, 2, (0, 0, 255), 5)
        elif self.selection_mode == "SHAPE_DRAW":
            for coord in self.shapedraw_points:
                cv2.circle(display, coord, 2, (0, 255, 255), 5)
            for coord in self.goal_points:
                cv2.circle(display, coord, 2, (0, 0, 255), 5)

        # Draw a line from each robot's live position to its assigned goal,
        # regardless of selection mode - this reflects the current
        # committed assignment, not the in-progress selection.
        for robot_id in self.robot_id_to_pose.keys():
            if robot_id not in self.robot_id_to_goal_pixel:
                continue  # no goal assigned yet for this robot
            x_world, y_world = self.robot_id_to_pose[robot_id]
            x_pos_pixel, y_pos_pixel = self.world_to_pixel(x_world, y_world, self.homography_matrix_inv)
            x_pos_pixel, y_pos_pixel = int(x_pos_pixel), int(y_pos_pixel)
            x_goal, y_goal = self.robot_id_to_goal_pixel[robot_id]

            cv2.line(display, (x_pos_pixel, y_pos_pixel), (x_goal, y_goal), (255, 0, 0), 1)

        cv2.imshow(self.window_name, display)

    def get_homography_file_path(self) -> str:
        """
        Locates the package share directory and constructs the file path for homography matrix

        Returns:
            str: The full absolute system path to the .npy file.
        """
        package_share_dir = get_package_share_directory('swarm_coppelia_drivers')
        file_path = os.path.join(package_share_dir, 'homography', 'homography.npy')

        self.get_logger().info(f'Located Homography matrix file at: {file_path}')
        return file_path

    def pixel_to_world(self, px, py, H):
        """
        Converts a single pixel coordinate to world coordinates using the
        forward homography matrix H (pixel -> world).
        """
        point = np.array([[[px, py]]], dtype=np.float32)
        world_point = cv2.perspectiveTransform(point, H)
        return world_point[0][0]  # (world_x, world_y)

    def world_to_pixel(self, wx, wy, H_inv):
        """
        Converts a single world coordinate back to pixel coordinates. Expects
        H_inv to already be the *inverse* homography (world -> pixel) - do not
        pass the forward matrix here, and do not invert inside this function,
        since the inverse is precomputed once in get_homography_matrix.
        """
        point = np.array([[[wx, wy]]], dtype=np.float32)
        pixel_point = cv2.perspectiveTransform(point, H_inv)
        return pixel_point[0][0]  # (px, py)

    def get_homography_matrix(self):
        """
        Load the homography matrix from homography.npy file
        """
        try:
            h_file_path = self.get_homography_file_path()
            self.homography_matrix = np.load(h_file_path)
            self.homography_matrix_inv = np.linalg.inv(self.homography_matrix)
        except Exception as e:
            self.get_logger().error(f"Failed to get homography matrix: \n{traceback.format_exc()}")

    def get_pose_callback(self, msg):
        """
        Subscription callback for /robot_pose_list. Rebuilds the live
        aruco_id <-> world-pose lookups from the latest pose array every time
        it's published. Does NOT trigger goal matching itself - matching only
        runs once, when a new goal set is finalized (see get_goal_points_world),
        so this just keeps the current-position data fresh for rendering and
        for whichever match_goal_points call comes next.
        """
        pose_array = msg
        self.robot_id_to_pose = {}
        self.robot_pose_to_id = {}

        for element in pose_array.robot_pose:
            self.robot_id_to_pose[element.aruco_id] = (element.x, element.y)
            self.robot_pose_to_id[(element.x, element.y)] = element.aruco_id

    def get_goal_points_world(self):
        """
        Converts the current pixel-space goal selection into world
        coordinates and, once a full goal set is available, triggers
        Hungarian matching against the robots' current positions.

        SELECT_N: only proceeds once exactly number_of_robots points have
        been clicked (goal_points_world is otherwise left empty).

        SHAPE_DRAW: resamples the drawn curve into number_of_robots points
        via select_goal_from_shape, then converts those to world coordinates.
        """
        self.goal_points_world = []
        if self.selection_mode == "SELECT_N" and len(self.goal_points) == self.number_of_robots:
            for point in self.goal_points:
                x, y = point
                x_world, y_world = self.pixel_to_world(x, y, self.homography_matrix)
                x_world, y_world = round(x_world, 2), round(y_world, 2)
                self.goal_points_world.append((x_world, y_world))
            self.get_logger().info(f"Goal Point Array World:{self.goal_points_world}")
            self.match_goal_points()

        elif self.selection_mode == "SHAPE_DRAW" and len(self.shapedraw_points) != 0:
            self.select_goal_from_shape()
            for point in self.goal_points:
                x, y = point
                x_world, y_world = self.pixel_to_world(x, y, self.homography_matrix)
                x_world, y_world = round(x_world, 2), round(y_world, 2)
                self.goal_points_world.append((x_world, y_world))
            self.get_logger().info(f"Goal Point Array:{self.goal_points}")
            self.get_logger().info(f"Goal Point Array World:{self.goal_points_world}")
            self.match_goal_points()

    def match_goal_points(self):
        """
        Runs Hungarian assignment once, matching each robot's current world
        position to a goal in self.goal_points_world such that total travel
        distance is minimized. Only proceeds if the number of currently-seen
        robots exactly matches the number of goal points; otherwise logs an
        error and leaves the previous assignment (if any) untouched.

        On success, populates robot_id_to_goal (world coords, for publishing)
        and robot_id_to_goal_pixel (pixel coords, for the debug overlay), then
        publishes the result.

        Note: this currently runs only once per finalized goal set (triggered
        from get_goal_points_world), not on every pose update. This means
        that if no robot poses are available yet when a goal set is
        finalized, the match will fail here and will NOT automatically retry
        once poses arrive - the user needs to reselect/redraw the goals.
        """
        self.robot_id_to_goal = {}
        self.robot_id_to_goal_pixel = {}
        robot_pose_list = list(self.robot_id_to_pose.values())

        if len(robot_pose_list) == len(self.goal_points_world):

            # Run matching steps
            cost_mat = self.compute_cost_matrix(robot_pose_list, self.goal_points_world)
            robot_indices, goal_indices = self.hungarian_algorithm(cost_mat)

            for r_idx, g_idx in zip(robot_indices, goal_indices):
                r_coord = robot_pose_list[r_idx]
                aruco_id = int(self.robot_pose_to_id[r_coord])
                g_coord = self.goal_points_world[g_idx]
                self.robot_id_to_goal[aruco_id] = g_coord
                self.robot_id_to_goal_pixel[aruco_id] = self.goal_points[g_idx]

            self.get_logger().info(f"Robot id to pose: {self.robot_id_to_pose}")
            self.get_logger().info(f"Robot id to goal: {self.robot_id_to_goal}")
            self.get_logger().info(f"Robot id to goal pixel: {self.robot_id_to_goal_pixel}")

            self.publish_robot_goal()

        else:
            self.get_logger().error("Cannot match goal points")

    def select_goal_from_shape(self):
        """
        Selects goal points evenly spaced (by arc length) along the curve drawn
        in self.shapedraw_points — one per robot — via linear interpolation,
        and stores the resulting pixel coordinates in self.goal_points.
        """
        if self.number_of_robots < 3:
            self.get_logger().error(
                f"select_goal_from_shape requires at least 3 robots, got {self.number_of_robots}"
            )
            return

        if len(self.shapedraw_points) < 2:
            self.get_logger().error("Not enough shape points drawn to select goals from.")
            return

        xs = np.array([p[0] for p in self.shapedraw_points], dtype=float)
        ys = np.array([p[1] for p in self.shapedraw_points], dtype=float)

        # Step 1: cumulative arc length up to each recorded point
        seg_lengths = np.hypot(np.diff(xs), np.diff(ys))
        cumulative_dist = np.concatenate(([0.0], np.cumsum(seg_lengths)))
        total_length = cumulative_dist[-1]

        if total_length == 0:
            self.get_logger().error("Drawn shape has zero length.")
            return

        # Step 2: n evenly spaced target distances along the curve (n-1 intervals)
        target_dists = np.linspace(0, total_length, self.number_of_robots)

        # Step 3: interpolate x and y independently at each target distance,
        # rather than snapping to the nearest recorded sample - this avoids
        # duplicate/colliding goal points in sparsely-sampled sections of
        # the curve (e.g. where the mouse moved quickly while drawing).
        goal_x = np.interp(target_dists, cumulative_dist, xs)
        goal_y = np.interp(target_dists, cumulative_dist, ys)

        self.goal_points = [(round(x), round(y)) for x, y in zip(goal_x, goal_y)]
        self.get_logger().info(f"Goal Points from Shape: {self.goal_points}")

    def compute_cost_matrix(self, robots, goals):
        """Computes a 2D matrix of squared Euclidean distances."""
        matrix = []
        for rx, ry in robots:
            row = []
            for gx, gy in goals:
                # Using squared distance to avoid floating-point issues
                dist = (rx - gx) ** 2 + (ry - gy) ** 2
                row.append(dist)
            matrix.append(row)
        return matrix

    def hungarian_algorithm(self, cost_matrix):
        """
        Solves the assignment problem via scipy's linear_sum_assignment
        (Jonker-Volgenant), returning the optimal robot-index <-> goal-index
        pairing that minimizes total cost. Note: this used to be a hand-rolled
        Hungarian implementation, but that version's greedy zero-assignment
        step wasn't a true augmenting-path algorithm and could infinite-loop
        on some inputs - scipy's implementation is correct and battle-tested.
        """
        cost_matrix = np.array(cost_matrix)
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        return list(row_ind), list(col_ind)

    def publish_robot_goal(self):
        """
        Packages the current robot_id_to_goal assignment into a
        RobotGoalArray message and publishes it on /robot_goal_list for
        robot_controller to consume.
        """
        goal_array = RobotGoalArray()
        for robot_id, goal in self.robot_id_to_goal.items():
            robot_goal = RobotGoal()
            robot_goal.aruco_id = int(robot_id)
            robot_goal.robot_name = self.robot_id_to_name[robot_id]
            x, y = goal
            robot_goal.x, robot_goal.y = round(float(x), 2), round(float(y), 2)
            goal_array.robot_goal.append(robot_goal)
        self.goal_publisher.publish(goal_array)


def main(args=None):
    # NO signal handling here since Python's default SIGINT handler is
    # relied on directly (see KeyboardInterrupt below) - note that
    # cv2.setMouseCallback in this node is known to interfere with SIGINT
    # delivery on shutdown (harmless; SIGTERM cleans it up after ~5s).
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    exit_code = 0

    try:
        node = GoalAllocation()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            # Re-render every loop iteration (not just on new frames) so the
            # window stays responsive to mouse events even between frames,
            # and pump the GUI event loop with waitKey.
            node.render()
            cv2.waitKey(1)

    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("Shutting down goal allocation via keyboard interrupt.")
            pass
    except Exception:
        exit_code = 1
        if node is not None:
            node.get_logger().error(f"Goal Allocation node crashed:\n{traceback.format_exc()}")
        else:
            print(f"Goal Allocation crashed during construction:\n{traceback.format_exc()}", flush=True)
    finally:
        cv2.destroyAllWindows()
        cv2.waitKey(1)  # let the GUI backend actually process the destroy event
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    main()