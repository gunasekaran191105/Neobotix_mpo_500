#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster

class MecanumDriveNode(Node):
    def __init__(self):
        super().__init__('mecanum_drive_node')
        
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        
        self.pub_w1 = self.create_publisher(Float64, '/neobotix_mpo_500/wheel_1/cmd_vel', 10)
        self.pub_w2 = self.create_publisher(Float64, '/neobotix_mpo_500/wheel_2/cmd_vel', 10)
        self.pub_w3 = self.create_publisher(Float64, '/neobotix_mpo_500/wheel_3/cmd_vel', 10)
        self.pub_w4 = self.create_publisher(Float64, '/neobotix_mpo_500/wheel_4/cmd_vel', 10)

        # Odometry Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.r = 0.125
        self.lx = 0.250
        self.ly = 0.29265  # Updated to match your clean CAD wheelbase!

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.02, self.update_odom) 

    def cmd_cb(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.wz = msg.angular.z

        k = (self.lx + self.ly)
        
        # Wheel 1: Front-Left
        w1 = (self.vx - self.vy - k * self.wz) / self.r
        # Wheel 2: Rear-Left
        w2 = (self.vx + self.vy - k * self.wz) / self.r
        # Wheel 3: Front-Right
        w3 = (self.vx + self.vy + k * self.wz) / self.r
        # Wheel 4: Rear-Right
        w4 = (self.vx - self.vy + k * self.wz) / self.r

        # All axes are Y-up, so no negative signs are needed!
        self.pub_w1.publish(Float64(data=w1))
        self.pub_w2.publish(Float64(data=w2))
        self.pub_w3.publish(Float64(data=w3)) 
        self.pub_w4.publish(Float64(data=w4))

    def update_odom(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0 or dt > 1.0:
            return

        # Odometry Math
        delta_x = (self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)) * dt
        delta_y = (self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)) * dt
        delta_th = self.wz * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_th

        # Publish /odom topic
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.wz
        self.odom_pub.publish(odom)

        # Publish TF transform
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = MecanumDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()