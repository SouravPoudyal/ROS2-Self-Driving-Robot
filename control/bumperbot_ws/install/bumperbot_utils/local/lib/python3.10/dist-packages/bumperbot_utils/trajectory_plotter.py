#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped 

class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__("trajectory_plotter")

        self.declare_parameter("odometry_topic", "bumperbot_controller/odom")
        self.odom_topic = self.get_parameter("odometry_topic")
        self.odom_subscriber = self.create_subscription(Odometry, self.odom_topic.value, self.odomCallback, 10)
        self.trajPublisher = self.create_publisher(Path, "bumperbot_controller/trajectory", 10)

        self.path_msgs_ = Path()

        self.get_logger().info('Trajectory node started')
    

    def odomCallback(self, msg: Odometry):

        self.path_msgs_.header.frame_id = msg.header.frame_id
        #PoseStamped message from the odometry message
        pose = PoseStamped()
        pose.header.frame_id = msg.header.frame_id
        pose.header.stamp = msg.header.stamp 
        pose.pose = msg.pose.pose

        self.path_msgs_.poses.append(pose)

        self.trajPublisher.publish(self.path_msgs_)

        self.get_logger().info('Published trajectory')

def main():
    rclpy.init()
    traj_plotter = TrajectoryPlotter()
    rclpy.spin(traj_plotter)

    traj_plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


