#!/usr/bin/python3
# Import necessary ROS 2 and other Python libraries
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped, Twist
import tf_transformations
import math
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import Pose, TransformStamped
import numpy as np

# Define a class for implementing Differential Drive Pure Pursuit using ROS 2
class DifferentialDrivePurePursuit(Node):
    # Constructor of the class
    def __init__(self):
        # Initialize the ROS 2 node
        super().__init__('differential_drive_pure_pursuit')
        # Create an action client for ComputePathToPose to get the path for the robot
        self.action_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        # Create a publisher for robot velocity commands
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        # Create a publisher for visualizing the goal point in RViz
        self.goal_marker_publisher = self.create_publisher(PoseStamped, 'goal_marker', 10)
        # Set lookahead distance for the Pure Pursuit algorithm
        self.lookahead_distance = 0.6
        # Set the threshold to determine if the goal is reached
        self.goal_threshold = 0.6
        # Initialize TF2 buffer and listener for pose transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # Create a timer to periodically run the pure pursuit controller method
        self.create_timer(0.05, self.pure_pursuit_controller)
        # Initialize path and current pose index
        self.path = None
        self.current_pose_index = 0
        self.total_path = None

    # Method implementing the Pure Pursuit control logic
    def pure_pursuit_controller(self):
        current_pose = self.get_robot_pose()
        if (current_pose != None):
            target_index = self.calculate_goal_point(self.path, current_pose, self.current_pose_index)
            linear_velocity, angular_velocity = self.calculate_velocities(current_pose, target_index)
            self.publish_velocity(linear_velocity, angular_velocity)

    # Method to send a navigation goal to the ComputePathToPose action server
    def send_goal(self, pose):
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = pose
        self.action_client.wait_for_server()
        self.future = self.action_client.send_goal_async(goal_msg)
        self.future.add_done_callback(self.goal_response_callback)

    # Callback for handling the response from the ComputePathToPose action server
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)

    # Callback for handling the path result from the ComputePathToPose action server
    def get_result_callback(self, future):
        result = future.result().result
        self.follow_path(result.path.poses)

    # Method to store the received path for following
    def follow_path(self, path):
        self.path = path
        self.total_path = len(self.path)


    # Method to calculate the goal point based on the lookahead distance
    def calculate_goal_point(self, path, robot_pose, start_index):
        # Implement Here !!!
        temp_pose_index = start_index
        target_point = path[temp_pose_index].pose.position
        curr_dist = self.distance_between_points(target_point, robot_pose.position)
        if curr_dist < self.lookahead_distance and temp_pose_index  < (self.total_path - 1):
            temp_pose_index += 1
            target_point = path[temp_pose_index].pose.position
            curr_dist = self.distance_between_points(target_point, robot_pose.position)
            self.get_logger().info(f'>> current_pose_index {self.current_pose_index}')

        self.current_pose_index = temp_pose_index
        return self.current_pose_index
            
    # Method to calculate linear and angular velocities to reach the goal point
    def calculate_velocities(self, robot_pose, goal_point):
        max_linear_velocity = 0.50  # Maximum linear velocity
        max_angular_velocity = 0.60  # Maximum angular velocity
        linear_velocity, angular_velocity = 0,0
        dp_x = self.path[goal_point].pose.position.x - robot_pose.position.x 
        dp_y = self.path[goal_point].pose.position.y - robot_pose.position.y 
        _, _, theta = tf_transformations.euler_from_quaternion([robot_pose.orientation.x,
                                                               robot_pose.orientation.y,
                                                               robot_pose.orientation.z,
                                                               robot_pose.orientation.w])
        e = np.arctan2(dp_y , dp_x ) - theta
        K = 0.50
        w = K * np.arctan2(np.sin(e), np.cos(e))

        if np.linalg.norm([dp_x, dp_y]) > self.goal_threshold:
            linear_velocity = max_linear_velocity
            angular_velocity = w
            if angular_velocity > max_angular_velocity:
                angular_velocity = max_angular_velocity
        else:
            linear_velocity = 0.0
            angular_velocity = 0.0

        return linear_velocity, angular_velocity
    
    # Method to publish the calculated velocity commands
    def publish_velocity(self, linear_vel, angular_vel):
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.velocity_publisher.publish(twist)

    # Method to check if the goal is reached
    def is_goal_reached(self, robot_pose, goal_pose):
        return self.distance_between_points(robot_pose.position, goal_pose.pose.position) <= self.goal_threshold

    # Utility method to calculate the Euclidean distance between two points
    def distance_between_points(self, point1, point2):
        return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)

    # Utility method to normalize an angle to the range [-pi, pi]
    def normalize_angle(self, angle):
        if angle > math.pi:
            angle -= 2.0 * math.pi
        if angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    # Method to get the current pose of the robot using TF2 transformations
    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            pose = Pose()
            pose.position.x = trans.transform.translation.x
            pose.position.y = trans.transform.translation.y
            pose.position.z = trans.transform.translation.z
            pose.orientation = trans.transform.rotation
            return pose
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error('Could not transform from base_link to map: %s' % str(e))
            return None
        
# Main function to initialize and run the ROS 2 node
def main(args=None):
    rclpy.init(args=args)
    node = DifferentialDrivePurePursuit()
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.pose.position.x = 2.0
    goal_pose.pose.position.y = -3.0
    node.send_goal(goal_pose)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
