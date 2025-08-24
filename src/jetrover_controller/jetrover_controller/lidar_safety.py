# lidar_safety.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class LidarSafety(Node):
    def __init__(self):
        super().__init__('lidar_safety')
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.publisher_ = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos)
        self.get_logger().info("LidarSafety node started. Subscribed to /scan with BEST_EFFORT QoS.")

    def lidar_callback(self, msg):
        self.get_logger().info(f"Received LaserScan with {len(msg.ranges)} points")

        # Filter valid distances
        ranges = [r for r in msg.ranges if 0.05 < r < 8.0 and r != float('inf') and not r != r]
        min_range = min(ranges) if ranges else float('inf')

        self.get_logger().info(f"Closest valid range: {min_range:.2f} m")

        if min_range < 0.2:
            self.get_logger().warn(f"Obstacle detected at {min_range:.2f} m! Stopping robot.")
            self.publisher_.publish(Twist())


def main():
    rclpy.init()
    node = LidarSafety()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
