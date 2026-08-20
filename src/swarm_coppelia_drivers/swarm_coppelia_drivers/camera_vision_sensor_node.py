import rclpy
from rclpy.node import Node
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
import numpy as np
import cv2
import cv2.aruco as aruco

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
        self.create_timer(0.1, self.camera_callback)


        # self.extract_yaml_data()

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

    def initialise_camera_handle(self):
        """
        This function initialises the coppeliasim handles for the vision sensor placed in the
        coppeliasim scene.
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
        Obtains the image captured by the vision sensor in the arena and converts it into a
        format opencv can understand
        """
        if self.simulation_state != "RUNNING":
            return

        ## Get image from coppeliasim vision sensor /overheadCamera
        img_data, resolution = self.sim.getVisionSensorImg(self.camera_handle)

        width, height = resolution[0], resolution[1]

        # Convert the raw buffer into a 1D NumPy array of unsigned 8-bit integers
        img_np = np.frombuffer(img_data, dtype=np.uint8)

        # Reshape the flat array into a 3D image matrix (Height x Width x RGB channels)
        img_np = img_np.reshape((height, width, 3))

        # CoppeliaSim's origin is at the bottom-left; OpenCV's is top-left.
        # Flip the image vertically (axis 0) to fix the orientation.
        img_np = cv2.flip(img_np, 0)

        # Convert from RGB (CoppeliaSim default) to BGR (OpenCV default)
        self.img_bgr = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)

    def camera_callback(self):
        """
        Camera callback function        
        """
        self.get_vision_sensor_image()

        self.detect_aruco_markers()

        
        cv2.imshow("Overhead Camera", self.img_bgr)
        cv2.waitKey(1)

    def detect_aruco_markers(self):
        """
        Detects all the aruco markers in the given frame and sets them up as a dictionary.
        {'id' :(top_left_corner, top_right_corner, bottom_right_corner, bottom_left_corner)}
        """

        # Convert to grayscale for cleaner detection thresholding
        gray = cv2.cvtColor(self.img_bgr, cv2.COLOR_BGR2GRAY)

        # Set up ArUco Dictionary (Match whatever type you used in CoppeliaSim, e.g., DICT_4X4_50)
        # Using the unified 4.x/5.x API format to avoid deprecation issues
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
        detector_params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)

        # Detect the markers
        corners, ids, rejected = detector.detectMarkers(gray)

        # 4. Check if any markers were actually found
        if ids is not None:
            # Flatten the multi-dimensional tracking array down to a simple 1D array
            ids_flat = ids.flatten()

            # Optional: Visual overlay for debug rendering
            cv2.aruco.drawDetectedMarkers(self.img_bgr, corners, ids)

            detected_arucos = {}

            for corner, marker_id in zip(corners, ids_flat):
                str_id = str(marker_id)
                box_corners = corner.reshape((4, 2))
                detected_arucos[str_id] = box_corners

            self.get_logger().info(f"Aruco Dict: {detected_arucos}")

            # for corner, marker_id in zip(corners, ids_flat):
            #     # Ensure the ID matches a string representation for your robot_lookup dictionary
            #     str_id = str(marker_id)

            #     if str_id in self.robot_id_to_name:
            #         robot_name = self.robot_id_to_name[str_id]
            #         self.get_logger().info(
            #             f"Detected {robot_name} (ArUco ID: {marker_id})"
            #         )

            #         # Extract the center point coordinates of the marker bounding box
            #         # corner layout: [[top_left, top_right, bottom_right, bottom_left]]
            #         box_corners = corner.reshape((4, 2))
            #         center_x = int(np.mean(box_corners[:, 0]))
            #         center_y = int(np.mean(box_corners[:, 1]))

            #         # Optional: Overlay text directly above the physical marker location
            #         cv2.putText(
            #             cv_image,
            #             robot_name,
            #             (center_x, center_y - 10),
            #             cv2.FONT_HERSHEY_SIMPLEX,
            #             0.5,
            #             (0, 255, 0),
            #             2,
            #         )
            #     else:
            #         self.get_logger().warn(
            #             f"Detected foreign ArUco ID: {marker_id} (Not in configuration lookup)"
            #         )
        else:
            self.get_logger().debug("No ArUco markers found in frame")




    
 
 
def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None

    try:
        node = CameraVisionSensorNode()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("Shutting down camera node via keyboard interrupt.")
            pass
    except Exception as e:
        print(f"Camera node failed: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
