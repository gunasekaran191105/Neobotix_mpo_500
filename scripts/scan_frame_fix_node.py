#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class ScanFrameFixNode(Node):
    def __init__(self):
        super().__init__('scan_frame_fix_node')
        
        # Best effort QoS profile is usually required for Gazebo sensor data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Subscribe to the raw bridge topic, publish to the clean /scan topic
        self.sub = self.create_subscription(LaserScan, '/scan_raw', self.scan_cb, qos_profile)
        self.pub = self.create_publisher(LaserScan, '/scan', 10)

    def scan_cb(self, msg):
        # Override the messy Gazebo frame ID with our clean URDF link
        msg.header.frame_id = 'lidar'
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanFrameFixNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
