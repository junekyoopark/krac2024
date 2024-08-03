#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped, Vector3
from std_msgs.msg import Bool
from px4_msgs.msg import VehicleCommand
import time
import math

class MissionOne(Node):
    def __init__(self):
        super().__init__('mission_one')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions
        self.position_subscriber = self.create_subscription(
            PoseStamped,
            '/px4_visualizer/vehicle_pose',
            self.position_callback,
            10
        )

        # Publishers
        self.velocity_publisher = self.create_publisher(
            Twist,
            '/offboard_velocity_cmd',
            qos_profile
        )

        self.arm_publisher = self.create_publisher(
            Bool,
            '/arm_message',
            qos_profile
        )

        self.vtol_publisher = self.create_publisher(
            Bool,
            '/vtol_message',
            qos_profile
        )

        # self.commander_publisher = self.create_publisher(
        #     VehicleCommand,
        #     '/fmu/in/vehicle_command',
        #     10
        # )


        # MAIN LOGIC 

        self.current_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        self.waypoints = [
            {'x': 0.0, 'y': 0.0, 'z': 8.0},
            {'x': 0.0, 'y': -150.0, 'z': 8.0},
            {'x': 0.0, 'y': 0.0, 'z': 8.0}
        ]

        self.curr_way_index = 0
        self.position_tolerance = 2
        self.vtol_count = 0

        time.sleep(5)
        self.get_logger().info("Main launched")
        self.arm_drone(True)
        time.sleep(10)
        self.navigate_waypoints()





    def position_callback(self, msg):
        self.current_position['x'] = msg.pose.position.x
        self.current_position['y'] = msg.pose.position.y
        self.current_position['z'] = msg.pose.position.z

    def arm_drone(self, arm):
        arm_msg = Bool()
        arm_msg.data = arm
        self.arm_publisher.publish(arm_msg)
        self.get_logger().info("Drone armed")

    def disarm_drone(self, arm):
        arm_msg = Bool()
        arm_msg.data = arm
        self.arm_publisher.publish(arm_msg)
        self.get_logger().info("Drone disarmed")

    def arm_vtol(self, varm):
        vtol_hmg = Bool()
        vtol_hmg.data = varm
        self.vtol_publisher.publish(vtol_hmg)
        self.get_logger().info("Go VTOL")

    def navigate_waypoints(self):
        self.wp_timer = self.create_timer(0.01, self.navigate_waypoint_callback)

    def navigate_waypoint_callback(self):
        if self.curr_way_index < len(self.waypoints):
            target = self.waypoints[self.curr_way_index]
            if self.is_waypoint_reached(target):
                self.get_logger().info(f"Waypoint {self.curr_way_index} reached")
                self.curr_way_index += 1
            elif (self.current_position['y'] < -10.0) and self.curr_way_index == 1 and self.vtol_count == 0:
                self.arm_vtol(True)
                self.vtol_count += 1
            else:
                twist = self.calculate_velocity_command(target)
                self.velocity_publisher.publish(twist)
                

        else:
            self.get_logger().info("All waypoints reached")
            self.disarm_drone(False)
            self.destroy_timer(self.wp_timer)

    def calculate_velocity_command(self, target):
        twist = Twist()
        kp = 0.2
        error_y = target['y'] - self.current_position['y']
        error_z = target['z'] - self.current_position['z']
        error_x = target['x'] - self.current_position['x']

        twist.linear.x = kp * error_x
        twist.linear.y = kp * -error_y
        twist.linear.z = kp * error_z

        max_speed = 5.0
        norm = math.sqrt(twist.linear.x**2 + twist.linear.y**2 + twist.linear.z**2)
        if norm > max_speed:
            twist.linear.x = (twist.linear.x / norm) * max_speed
            twist.linear.y = (twist.linear.y / norm) * max_speed
            twist.linear.z = (twist.linear.z / norm) * max_speed

        twist.angular = Vector3(x=0.0, y=0.0, z=0.0)

        return twist

    def is_waypoint_reached(self, target):
        x_dist = abs(target['x'] - self.current_position['x'])
        y_dist = abs(target['y'] - self.current_position['y'])
        z_dist = abs(target['z'] - self.current_position['z'])

        return (x_dist < self.position_tolerance and
                y_dist < self.position_tolerance and
                z_dist < self.position_tolerance)

def main(args=None):
    rclpy.init(args=args)
    mission1node = MissionOne()
    rclpy.spin(mission1node)
    mission1node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()