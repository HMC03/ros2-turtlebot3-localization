#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose, PoseWithCovarianceStamped
import math

from tb3_localization.mcl.map_loader import MapLoader
from tb3_localization.mcl.sensor_model import LikelihoodFieldModel
from tb3_localization.mcl.particle_filter import ParticleFilter


class MCLNode(Node):
    def __init__(self):
        super().__init__('mcl_node')
        
        self.map = MapLoader()
        self.sensor = LikelihoodFieldModel(self.map, sigma_hit=0.18)
        self.pf = ParticleFilter(
            map_loader=self.map,
            sensor_model=self.sensor,
            N=800,
            seed=42
        )
        
        self.last_odom = None
        
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        
        self.particle_pub = self.create_publisher(PoseArray, '/particle_cloud', 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/estimated_pose', 10)
        self.create_timer(0.1, self.publish)
        
        self.get_logger().info("ONE TRUE PARTICLE FILTER — 800 PARTICLES — READY")

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        th = 2 * math.atan2(q.z, q.w)
        
        if self.last_odom is not None:
            self.pf.motion_update(self.last_odom, (x, y, th))
        self.last_odom = (x, y, th)

    def scan_cb(self, msg):
        if self.last_odom is None:
            return
        self.pf.measurement_update(msg.ranges)

    def publish(self):
        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = 'map'
        
        for p in self.pf.get_pose_array():
            pose = Pose()
            pose.position.x = p[0]
            pose.position.y = p[1]
            pose.orientation.z = math.sin(p[2]/2)
            pose.orientation.w = math.cos(p[2]/2)
            pa.poses.append(pose)
        self.particle_pub.publish(pa)

        # Best estimate
        x, y, th = self.pf.get_best_pose()
        est = PoseWithCovarianceStamped()
        est.header = pa.header
        est.pose.pose.position.x = x
        est.pose.pose.position.y = y
        est.pose.pose.orientation.z = math.sin(th/2)
        est.pose.pose.orientation.w = math.cos(th/2)
        est.pose.covariance[0] = 0.001
        est.pose.covariance[7] = 0.001
        est.pose.covariance[35] = 0.001
        self.pose_pub.publish(est)


def main():
    rclpy.init()
    node = MCLNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()