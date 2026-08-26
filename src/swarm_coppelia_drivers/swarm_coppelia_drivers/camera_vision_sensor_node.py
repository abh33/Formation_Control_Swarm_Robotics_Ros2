import rclpy
from rclpy.node import Node
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
import numpy as np
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import traceback
from swarm_interfaces.msg import RobotPose, RobotPoseArray
from ament_index_python.packages import get_package_share_directory
import os


class CameraVisionSensorNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("camera_vision_sensor_node", automatically_declare_parameters_from_overrides=True) # MODIFY NAME
        # self.declare_parameter('robot_lookup', '')
        self.get_logger().info("Camera Node has started!")

        # Step 1: Establish connection to CoppeliaSim
        self.connect_to_coppelia()

        # Step 2: Create a subscriber for /simulation_state
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.simulation_state = None
        self.simulation_state_subscriber = self.create_subscription(String, '/simulation_state', self.update_simulation_state, qos)

        # Step 3: Get the overhead camera handle from the coppeliasim scene
        self.initialise_camera_handle()

        # # Step 4: Get the robot lookup dictionary passed as parameter to the node.
        self.robot_id_to_name = {}
        self.extract_lookup_data()

        ## Step 5: Create a timer to trigger the camera callback every 0.1 seconds
        self.detected_arucos = {}
        self.aruco_pose_dict = {}
        self.create_timer(0.1, self.camera_callback)

        ## Step 6: Create a publisher to publish raw image to a camera topic
        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, '/camera/image_raw', 10)

        ## Step 7: Create a publisher to publish robot pose to a topic /robot_pose_list
        self.pose_publisher = self.create_publisher(RobotPoseArray, '/robot_pose_list', 10)

        self.counter = 0

        ## Step 8: Homography        
        self.homography_matrix = None
        self.get_homography_matrix()



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
        Subscription callback for /simulation_state. Stores the latest published
        state string (e.g. "RUNNING", "STOPPED") so other methods can gate their
        behavior on it without querying CoppeliaSim directly.
        """
        self.simulation_state = msg.data

    def initialise_camera_handle(self):
        """
        Blocks (via repeated spin_once calls) until /simulation_state reports
        "RUNNING", then resolves and caches the CoppeliaSim object handle for
        the overhead vision sensor. Raises if the handle can't be resolved,
        since no frame can be captured without it.
        """
        while rclpy.ok() and (self.simulation_state is None or self.simulation_state != "RUNNING"):
            rclpy.spin_once(self, timeout_sec=0.5)

        try:
            self.camera_handle = self.sim.getObject("/arena/overheadCamera")
            if self.camera_handle == -1:
                raise RuntimeError("Camera handle not valid")
            self.get_logger().info("Camera handle initialised correctly")
        except Exception as e:
            self.get_logger().error(f"Failed to get object handle: {str(e)}")
            raise e

    def extract_lookup_data(self) -> None:
        """
        Reads the robot_lookup parameter block (ArUco ID -> robot name mapping)
        passed in via the launch-time YAML config, and rebuilds it into a plain
        {id_string: robot_name} dict for fast lookup during pose publishing.
        """
        # 1. Fetch the flattened 'robot_lookup' dictionary parameters
        raw_lookup = self.get_parameters_by_prefix('robot_lookup')
        
        # 2. Reconstruct the clean dictionary
        for key, param in raw_lookup.items():
            # Remove the 'id_' prefix to get your clean integer-like string back
            clean_id = key.replace('id_', '')
            self.robot_id_to_name[clean_id] = param.value

        self.get_logger().info(f"Successfully loaded lookup: {self.robot_id_to_name}")

    def get_vision_sensor_image(self) -> None:
        """
        Pulls the current frame from CoppeliaSim's overhead vision sensor,
        reshapes the raw byte buffer into an (H, W, 3) array, flips it vertically
        to align CoppeliaSim's bottom-left image origin with OpenCV's top-left
        convention, and converts RGB to BGR. Stores the result in self.img_bgr
        for use by every downstream step in this tick's callback.
        """
        ## Get image from coppeliasim vision sensor /overheadCamera
        img_data, resolution = self.sim.getVisionSensorImg(self.camera_handle)

        width, height = resolution[0], resolution[1]

        # 1. Your original buffer extraction (KEEP THIS - it matches your 512x512 byte stream)
        img_np = np.frombuffer(img_data, dtype=np.uint8)

        # 2. Your original reshape mapping
        img_np = img_np.reshape((height, width, 3))

        # 3. Your original coordinate flipping layout
        img_np = cv2.flip(img_np, 0)

        # 4. THE ONLY CHANGED LINE: Explicitly enforce uint8 array formatting on the BGR output matrix
        # This completely strips out the underlying float signature that caused KeyError: 16 inside cv_bridge
        self.img_bgr = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR).astype(np.uint8)


    def publish_raw_image_frame(self):
        """
        Wraps the current self.img_bgr frame as a sensor_msgs/Image (via
        cv_bridge) and publishes it to /camera/image_raw, timestamped with the
        current ROS clock, for external viewing/debugging (e.g. rqt_image_view).
        """

        image_msg = self.bridge.cv2_to_imgmsg(self.img_bgr, encoding='bgr8')

        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = 'overhead_camera'
        self.image_publisher.publish(image_msg)


    def camera_callback(self):
        """
        Main per-tick pipeline, run on a timer once the sim is RUNNING: capture
        a frame, publish it raw, detect ArUco markers, compute each detected
        robot's pose, publish the aggregate pose array, and render a local debug
        window with detection overlays.
        """
        if self.simulation_state != "RUNNING":
            return
        self.get_vision_sensor_image()

        self.publish_raw_image_frame()        

        self.detect_aruco_markers()

        self.detect_aruco_pose()

        self.publish_robot_pose()

        # self.homography_calculations()
        
        cv2.imshow("Overhead Camera", self.img_bgr)
        cv2.waitKey(1)

    def detect_aruco_markers(self):
        """
        Runs ArUco marker detection on the current grayscale frame and rebuilds
        self.detected_arucos as {aruco_id: 4x2 corner array} for every marker
        seen THIS tick. Reset from scratch every call so a marker that's no
        longer visible doesn't linger from a previous frame.
        """

        # Convert to grayscale for cleaner detection thresholding
        gray = cv2.cvtColor(self.img_bgr, cv2.COLOR_BGR2GRAY)

        # Set up ArUco Dictionary (Match whatever type you used in CoppeliaSim, e.g., DICT_4X4_50)
        # Using the unified 4.x/5.x API format to avoid deprecation issues
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
        detector_params = cv2.aruco.DetectorParameters()
        detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        detector_params.adaptiveThreshWinSizeMin = 3
        detector_params.adaptiveThreshWinSizeMax = 23
        detector_params.adaptiveThreshWinSizeStep = 4
        detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)

        # Detect the markers
        corners, ids, rejected = detector.detectMarkers(gray)

        # 4. Check if any markers were actually found
        self.detected_arucos = {}
        if ids is not None:
            # Flatten the multi-dimensional tracking array down to a simple 1D array
            ids_flat = ids.flatten()

            # Optional: Visual overlay for debug rendering
            cv2.aruco.drawDetectedMarkers(self.img_bgr, corners, ids)

            for corner, marker_id in zip(corners, ids_flat):
                self.detected_arucos[int(marker_id)] = corner.reshape((4, 2))
        else:
            self.get_logger().debug("No ArUco markers found in frame")

    def detect_aruco_pose(self):
        """
        Converts each detected marker's corner points into a 2D pose: center
        position (mean of the four corners) and heading angle (from the vector
        between the top-left and top-right corners, in degrees). Rebuilds
        self.aruco_pose_dict as {aruco_id: (x, y, theta_degrees)} for markers
        detected this tick only.
        """
        try:
            self.aruco_pose_dict = {}
            # self.get_logger().info(f"detected_arucos.keys(): {self.detected_arucos.keys()}")

            if self.detected_arucos and self.homography_matrix is not None:
                for detected in self.detected_arucos.keys():

                    aruco_id = detected
                    corners = self.detected_arucos[aruco_id]
                    center_x = int(np.mean(corners[:, 0]))
                    center_y = int(np.mean(corners[:, 1]))
                    wpos_x, wpos_y = self.pixel_to_world(center_x, center_y, self.homography_matrix)
                    wpos_x, wpos_y= round(float(wpos_x), 2), round(float(wpos_y),2)
                    

                    # Calculate orientation angle (theta) using top-left and top-right corners of aruco marker
                    # and converting them to world coordinates
                    # top_left = corners[0], top_right = corners[1]
                    top_left_world = self.pixel_to_world(corners[0][0], corners[0][1], self.homography_matrix)
                    top_right_world = self.pixel_to_world(corners[1][0], corners[1][1], self.homography_matrix)

                    world_dx = top_right_world[0] - top_left_world[0]
                    world_dy = top_right_world[1] - top_left_world[1]
                    theta_degrees = round(float(np.degrees(np.arctan2(world_dy, world_dx))), 2)
                    theta_corrected = np.degrees(np.arctan2(np.sin(np.radians(theta_degrees + 90)), np.cos(np.radians(theta_degrees + 90))))
                    self.aruco_pose_dict[aruco_id] = (wpos_x, wpos_y, theta_corrected)

                # self.get_logger().info(f"Aruco_pose_dict: {self.aruco_pose_dict}")

            else:
                self.get_logger().info("Aruco dictionary is empty")
        except Exception as e:
            self.get_logger().error(f"Failed to compute ArUco poses: \n{traceback.format_exc()}")

    def publish_robot_pose(self):
        """
        Assembles a RobotPoseArray covering every robot in the configured
        lookup table, pairing each with its pose from self.aruco_pose_dict when
        available. Robots whose marker wasn't detected this tick are still
        included, marked as not-detected, so downstream consumers can
        distinguish a stale/missing reading from a real one.
        """
        pose_array = RobotPoseArray()
        for id_str, name in self.robot_id_to_name.items():
            id_int = int(id_str)
            pose = RobotPose()
            pose.aruco_id = id_int
            pose.robot_name = name
            if id_int in self.aruco_pose_dict:
                x, y, theta = self.aruco_pose_dict[id_int]
                pose.x, pose.y, pose.theta = x, y, theta
                pose.detected = True
            else:
                pose.x, pose.y, pose.theta = 0.0, 0.0, 0.0
                pose.detected = False
            pose_array.robot_pose.append(pose)
        self.pose_publisher.publish(pose_array)

    # def homography_calculations(self):
    #     aruco_list = []
    #     pixel_pos_list = []
    #     aruco_keys_sorted = sorted(self.aruco_pose_dict.keys())
    #     world_pos_list = []
    #     for id in aruco_keys_sorted:
    #         aruco_list.append(id)
    #         temp = (self.aruco_pose_dict[id][0], self.aruco_pose_dict[id][1])
    #         temp = list(temp)
    #         pixel_pos_list.append(temp)

    #     world_pos_list = [[0.0, 0.0],[-1.3, -1.3],[1.3, 1.3],[-1.3, 1.3],[1.3, -1.3], [-1.3, 0.0],[1.3, 0.0],[0.0, 1.3],[0.0, -1.3],[-0.65, -0.65], [0.65, 0.65], [-0.65, 0.65], [0.65, -0.65], [-0.65, 0.0], [0.65, 0.0], [0.0, 0.65], [0.0, -0.65]]

    #     pixel_pos_np = np.array(pixel_pos_list, dtype=np.float32)
    #     world_pos_np = np.array(world_pos_list, dtype=np.float32)

        # self.get_logger().info(f"{pixel_pos_np}")
        # self.get_logger().info(f"{world_pos_np}")

        # H, mask = cv2.findHomography(pixel_pos_np, world_pos_np, method=cv2.RANSAC)

        # self.counter = self.counter + 1
        # self.get_logger().info(f"Counter: {self.counter}")
        # self.get_logger().info(f"Homography: {H}")
        # self.get_logger().info(f"Mask: {mask}")
        # if self.counter == 201:
        #     np.save('homography.npy', H)

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
        point = np.array([[[px, py]]], dtype=np.float32)
        world_point = cv2.perspectiveTransform(point, H)
        return world_point[0][0]  # (world_x, world_y)

    def get_homography_matrix(self):
        """
        Load the homography matrix from homography.npy file
        """
        try:
            h_file_path = self.get_homography_file_path()
            self.homography_matrix = np.load(h_file_path)
        except Exception as e:
            self.get_logger().error(f"Failed to get homography matrix: \n{traceback.format_exc()}")







    
 
 
def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    exit_code = 0

    try:
        node = CameraVisionSensorNode()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("Shutting down camera node via keyboard interrupt.")
            pass
    except Exception:
        exit_code = 1
        if node is not None:
            node.get_logger().error(f"Camera node crashed:\n{traceback.format_exc()}")
        else:
            print(f"Camera node crashed during construction:\n{traceback.format_exc()}", flush=True)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code
 
 
if __name__ == "__main__":
    main()
