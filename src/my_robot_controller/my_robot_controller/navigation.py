#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from turtlesim.msg import Pose
import tf_transformations
import time


class TurtleNavigationNode(Node):
    
    def __init__(self):
        super().__init__("navigation")
        self.get_logger().info("our navigation is started")
        
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10)
        
        self.goal_pose_publisher = self.create_publisher(
            PoseStamped, "/goal_pose", 10)
        
        self.odom_listener = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10)
        
        ############# [Initial Location] ############
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.pose.pose.position.x = 0.0
        initial_pose.pose.pose.position.y = 0.0
                
        qq = tf_transformations.quaternion_from_euler(0,0,0)# x, y, z or Roll Pitch Yaw
        initial_pose.pose.pose.orientation.x = qq[0]
        initial_pose.pose.pose.orientation.y = qq[1]
        initial_pose.pose.pose.orientation.z = qq[2]
        initial_pose.pose.pose.orientation.w = qq[3]
        self.initial_pose_publisher.publish(initial_pose)
        #################################
        # time.sleep(1)
        # ############# [Destination] ############
        # goal = PoseStamped()
        # goal.header.frame_id = 'map'
        # goal.pose.position.x = 3.5
        # goal.pose.position.y = 0.0
        # qq = tf_transformations.quaternion_from_euler(0,0,1.57)# x, y, z or Roll Pitch Yaw
        # goal.pose.orientation.x = qq[0]
        # goal.pose.orientation.y = qq[1]
        # goal.pose.orientation.z = qq[2]
        # goal.pose.orientation.w = qq[3]
        # self.goal_pose_publisher.publish(goal)
        
        
        # Initialize goal poses as dictionaries {x, y, w}
        self.goal_poses = []
        self.goal_poses.append({'x': 1.0, 'y': 1.0, 'w': 1.0})
        self.goal_poses.append({'x': 2.0, 'y': 2.0, 'w': 1.0})
        self.goal_poses.append({'x': 3.0, 'y': 3.0, 'w': 1.0})
        
        
    def odom_callback(self, msg: Odometry):
        # Check if current goal pose is reached
        current_pose = msg.pose.pose
        goal_pose = self.goal_poses[self.current_goal_index]
        distance_to_goal = ((current_pose.position.x - goal_pose['x']) ** 2 +
                            (current_pose.position.y - goal_pose['y']) ** 2) ** 0.5
        if distance_to_goal < 0.5:  # You can adjust this threshold
            self.publish_next_goal()
            
    def publish_next_goal(self):
        # Check if there are more goals to explore
        if self.current_goal_index < len(self.goal_poses) - 1:
            self.current_goal_index += 1
            pose_msg = PoseStamped()
            pose_msg.pose.position.x = self.goal_poses[self.current_goal_index]['x']
            pose_msg.pose.position.y = self.goal_poses[self.current_goal_index]['y']
            pose_msg.pose.orientation.w = self.goal_poses[self.current_goal_index]['w']
            self.goal_pub.publish(pose_msg)
            self.get_logger().info("Published next goal: {}".format(self.current_goal_index))
        else:
            self.get_logger().info("All goals explored!")


def main(args=None):
    rclpy.init(args=args)
    node = TurtleNavigationNode()
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()