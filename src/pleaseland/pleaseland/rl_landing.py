import rclpy
from rclpy.node import Node
import math
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from px4_msgs.msg import VehicleOdometry
import onnxruntime
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import rospkg
import os


class RLControl(Node):
    '''
    Class for precisely landing the UAV using reinforcement learning.
    Training was conducted in the Unity environment. The learned parameters are saved in an ONNX file.
    Algorithms: 
        Soft Actor-Critic (SAC)
    '''
    def __init__(self):
        super().__init__('rl_aruco_control_node')  # Initialize the ROS2 node

        # Initialize QoS Profile for PX4
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Initialize ROS2 variables
        self.current_vel = None
        self.relative_dis = Float32MultiArray()

        # Subscribers
        self.relative_dis_sub = self.create_subscription(
            Float32MultiArray, 
            '/relative_distance', 
            self.relative_dis_cb, 
            qos_profile
        )
        self.vel_sub = self.create_subscription(
            VehicleOdometry, 
            '/fmu/out/vehicle_odometry', 
            self.vel_cb, 
            qos_profile
        )

        # Publisher
        self.twist_pub = self.create_publisher(Twist, 'output_twist', qos_profile)

        self.scale = 0.2
        self.z_offset = 0.0

        # Load ONNX model

        self.rospack = rospkg.RosPack()
        # self.onnx_model_dir = self.rospack.get_path('pleaseland')
        # self.model_path = (self.rospack+'/onnx_models/DroneLanding-5000091.onnx')
        self.model_path = '/home/john/krac2024/src/pleaseland/onnx_models/DroneLanding-legacykrac2023.onnx'
        self.model = onnxruntime.InferenceSession(self.model_path)

        self.yaw = 90 * math.pi / 180  # Desired yaw angle in radians.
        
        # Initialization
        self.relative_dis.data = [0.0, 0.0, 0.0]

        # Timer for the action callback
        self.create_timer(0.05, self.action)

    def relative_dis_cb(self, msg):
        self.relative_dis = msg

    def vel_cb(self, msg):
        self.current_vel = msg.velocity  # This should be a 3-element array [x, y, z]

    def get_state(self):
        '''
        Transform the current state to match the expected input format for the RL model.
        Unity uses a different coordinate system than ROS (ENU).
        '''
        state = []
        # self.get_logger().info(f"Received velocity: {self.current_vel}")

        if self.current_vel is not None and len(self.relative_dis.data) >= 3:
            # MAVROS (ENU) to Unity coordinate system transformation:
            # ENU -> Unity: x (East) -> y, y (North) -> z, z (Up) -> x
            state.append(self.current_vel[1])  # Unity x <- ROS y (North)
            state.append(self.current_vel[2])  # Unity y <- ROS z (Up)
            state.append(self.current_vel[0])  # Unity z <- ROS x (East)

            state.append(self.relative_dis.data[0])  # x distance (Unity x <- ROS x (East))
            state.append(self.relative_dis.data[2])  # z height (Unity y <- ROS z (Up))
            state.append(self.relative_dis.data[1])  # y distance (Unity z <- ROS y (North))
        
        # Check if the state is not fully populated
        if len(state) != 6:
            # self.get_logger().warning('Incomplete state data for the RL model. Skipping inference.')
            return None
        
        return state

    def action(self):
        '''
        Control logic based on marker detection:
        1) If marker is not detected (NaN): Position control.
        2) If marker is detected: Velocity control using the RL model.
        '''
        state = self.get_state()
        twist_msg = Twist()

        if state is None:
            # Incomplete state data, skip inference
            return

        if np.isnan(self.relative_dis.data[0]):
            # Marker not detected, perform position control (hover in place, reduce altitude slightly)
            twist_msg.linear.x = 0
            twist_msg.linear.y = 0
            twist_msg.linear.z = -0.4  # Descend slowly

            twist_msg.angular.z = self.yaw
            self.twist_pub.publish(twist_msg)
        else:
            # Marker detected, perform velocity control using RL model
            ort_inputs = {self.model.get_inputs()[0].name: [state]}
            action = self.model.run(None, ort_inputs)
            # self.get_logger().info(f"Action UNTOUCHED: {action}")

            # Transform and scale action to match the control frame
            action = np.multiply(action[2][0], [1, 1, 0.8])
            action = action * self.scale
            self.get_logger().info(f"Action: {action}")

            # Unity to MAVROS (ENU) coordinate transformation:
            # Unity x -> ROS y (North), Unity y -> ROS z (Up), Unity z -> ROS x (East)
            twist_msg.linear.x = action[2]  # ROS x (East) <- Unity z
            twist_msg.linear.y = action[0]  # ROS y (North) <- Unity x
            twist_msg.linear.z = action[1]  # ROS z (Up) <- Unity y
            twist_msg.angular.z = self.yaw
            self.twist_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    rl_control_node_handler = RLControl()

    rclpy.spin(rl_control_node_handler)

    rl_control_node_handler.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
