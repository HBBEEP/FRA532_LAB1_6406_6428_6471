#!/usr/bin/python3
# Import necessary ROS 2 and other Python libraries
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np

class GlobalPlanner(Node):
    def __init__(self):
        super().__init__('global_planner')
        self.action_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.path = None
        self.total_path = None
        self.path_pub = self.create_publisher(Path, 'robot_path', 10)

    def send_goal(self, pose):
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = pose
        self.action_client.wait_for_server()
        self.future = self.action_client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.follow_path(result.path.poses)

    def follow_path(self, path):
        self.path = path
        path_msg = Path()
        path_msg.header.frame_id = "map"
        if (path != None):
            print(path)

            for i in range(len(self.path)):
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = "map"
                pose.pose.position.x = self.path[i].pose.position.x
                pose.pose.position.y = self.path[i].pose.position.y
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)
            
            self.path_pub.publish(path_msg)        

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlanner()
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.pose.position.x =  5.00 #0.496
    goal_pose.pose.position.y =  5.00 #5.780
    node.send_goal(goal_pose)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
