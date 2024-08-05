import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from px4_msgs.msg import VehicleOdometry

class OdometryTFBroadcaster(Node):
    def __init__(self):
        super().__init__('odometry_tf_broadcaster')
        # Subscribe to the vehicle odometry topic
        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.subscription = self.create_subscription(
            VehicleOdometry,
            'fmu/out/vehicle_odometry',
            self.handle_vehicle_odometry,
            qos_profile)
        self.br = tf2_ros.TransformBroadcaster(self)
        self.get_logger().info("Vehicle Odometry TF broadcaster has been started")

    def handle_vehicle_odometry(self, msg):
        t = TransformStamped()

        # Fill the TransformStamped data
        t.header.stamp = self.get_clock().now().to_msg()  # Adapt as needed for real timestamp
        t.header.frame_id = 'map'  # Assume map as the reference frame
        t.child_frame_id = 'odom'  # Assume odom as the child frame
        # Ensure to handle NaNs and set default values if necessary
        if not any(nan in msg.position for nan in (float('nan'),)):
            t.transform.translation.x = float(msg.position[0])
            t.transform.translation.y = float(msg.position[1])
            t.transform.translation.z = float(msg.position[2])
        if not any(nan in msg.q for nan in (float('nan'),)):
            t.transform.rotation.x = float(msg.q[0])
            t.transform.rotation.y = float(msg.q[1])
            t.transform.rotation.z = float(msg.q[2])
            t.transform.rotation.w = float(msg.q[3])

        # Broadcast the transform
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
