import numpy as np
import cv2
import cv2.aruco as aruco
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy


class DistanceToAruco(Node):
    def __init__(self):
        super().__init__('aruco_vio')

        # Initialize QoS Profile for PX4
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.bridge = CvBridge()
        self.measured_xy = Point()
        self.dis = Float32MultiArray()
        self.mission = 0

        self.image_pub = self.create_publisher(Image, '/cv_image', qos_profile)
        self.distance_pub = self.create_publisher(Float32MultiArray, '/relative_distance', qos_profile)
        self.measured_xy_pub = self.create_publisher(Point, '/measured_xy', qos_profile)

        self.cameraMatrix = np.array([[1075.150341, 0.0, 640.0], 
                                      [0.0, 1075.150341, 360.0], 
                                      [0.0, 0.0, 1.0]])
        self.distortion = np.array([[0.176173, -0.394370, -0.003991, 0.005109]])

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        self.parameters = aruco.DetectorParameters_create()

        self.inner_marker_size = 0.08 #0.08
        self.mid_marker_size = 0.45
        self.outer_marker_size = 1.92
        self.inner_objp = np.array([[0, 0, 0], [0, self.inner_marker_size, 0], 
                                    [self.inner_marker_size, self.inner_marker_size, 0], 
                                    [self.inner_marker_size, 0, 0]], dtype=np.float32)
        self.mid_objp = np.array([[0, 0, 0], [0, self.mid_marker_size, 0], 
                                  [self.mid_marker_size, self.mid_marker_size, 0], 
                                  [self.mid_marker_size, 0, 0]], dtype=np.float32)
        self.outer_objp = np.array([[0, 0, 0], [0, self.outer_marker_size, 0], 
                                    [self.outer_marker_size, self.outer_marker_size, 0], 
                                    [self.outer_marker_size, 0, 0]], dtype=np.float32)

        # Subscribe to the image topic instead of using a VideoCapture
        self.image_sub = self.create_subscription(Image, '/image', self.image_to_distance_cb, 1)

    def image_to_distance_cb(self, msg):
        # Convert the ROS Image message to a cv2 image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        height, width, _ = cv_image.shape
        cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(cv_image_gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            self.get_logger().info(f"Detected marker IDs: {ids.flatten()}")
            aruco.drawDetectedMarkers(cv_image, corners)

            # Assume inner_id, mid_id, and outer_id logic as before
            tvec = None
            if 2 in ids:
                idx = list(ids.flatten()).index(2)
                tmp_corner = corners[idx][0]
                _, rvec, tvec = cv2.solvePnP(self.inner_objp, tmp_corner, self.cameraMatrix, self.distortion)
            elif 1 in ids:
                idx = list(ids.flatten()).index(1)
                tmp_corner = corners[idx][0]
                _, rvec, tvec = cv2.solvePnP(self.mid_objp, tmp_corner, self.cameraMatrix, self.distortion)
            elif 0 in ids:
                idx = list(ids.flatten()).index(0)
                tmp_corner = corners[idx][0]
                _, rvec, tvec = cv2.solvePnP(self.outer_objp, tmp_corner, self.cameraMatrix, self.distortion)

            if tvec is not None:
                x, y, z = tvec[0][0], tvec[1][0], tvec[2][0]
                self.dis.data = [x, y, z]
                self.distance_pub.publish(self.dis)
                self.get_logger().info(f"Distance: x={x:.2f}, y={y:.2f}, z={z:.2f}")

                x_center_px = np.mean(tmp_corner[:, 0])
                y_center_px = np.mean(tmp_corner[:, 1])
                self.measured_xy.x = float(x_center_px)
                self.measured_xy.y = float(y_center_px)
                self.measured_xy_pub.publish(self.measured_xy)

                cv2.rectangle(cv_image, (int(x_center_px - 30), int(y_center_px - 30)),
                                (int(x_center_px + 30), int(y_center_px + 30)), (255, 0, 0), 2)
                cv2.putText(cv_image, "Measured Position", (int(x_center_px + 30), int(y_center_px - 30)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 191, 255), 2)
            else:
                self.get_logger().warn("No valid ArUco marker detected for pose estimation")
        else:
            self.get_logger().info("No ArUco markers detected.")

        # Display the image with detected markers in a window
        cv2.imshow('Aruco Detection', cv_image)
        cv2.waitKey(1)

        # Node publish - cv_image
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "rgb8"))

def main(args=None):
    rclpy.init(args=args)
    distance_to_aruco_node = DistanceToAruco()
    rclpy.spin(distance_to_aruco_node)
    distance_to_aruco_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
