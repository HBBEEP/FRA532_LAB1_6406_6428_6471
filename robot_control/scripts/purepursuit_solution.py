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

    # Method implementing the Pure Pursuit control logic
    def pure_pursuit_controller(self):
        if self.path is not None and self.current_pose_index < len(self.path):
            # Get the current target pose from the path
            current_pose = self.path[self.current_pose_index]
            # Retrieve the robot's current pose
            robot_pose = self.get_robot_pose()
            if robot_pose is not None:
                # Calculate the goal point based on the robot's pose and the path
                goal_point = self.calculate_goal_point(self.path, robot_pose, self.current_pose_index)
                if goal_point is not None:
                    # Calculate the necessary velocities to move towards the goal point
                    linear_vel, angular_vel = self.calculate_velocities(robot_pose, goal_point)
                    # Publish the velocity commands
                    self.publish_velocity(linear_vel, angular_vel)
                    # Check if the current target pose is reached
                    if self.is_goal_reached(robot_pose, current_pose):
                        self.current_pose_index += 1

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
        self.get_logger().info('Path received')
        self.follow_path(result.path.poses)

    # Method to store the received path for following
    def follow_path(self, path):
        self.path = path

    # Method to calculate the goal point based on the lookahead distance
    def calculate_goal_point(self, path, robot_pose, start_index):
        for i in range(start_index, len(path)):
            if self.distance_between_points(robot_pose.position, path[i].pose.position) >= self.lookahead_distance:
                # Publish the goal point for visualization in RViz
                goal_marker = PoseStamped()
                goal_marker.header.frame_id = "map"
                goal_marker.pose.position = path[i].pose.position
                self.goal_marker_publisher.publish(goal_marker)
                return path[i]
        return None

    # Method to calculate linear and angular velocities to reach the goal point
    def calculate_velocities(self, robot_pose, goal_point):
        max_linear_velocity = 0.50  # Maximum linear velocity
        max_angular_velocity = 0.60  # Maximum angular velocity
        # Calculate the angle to the goal point
        angle_to_goal = math.atan2(goal_point.pose.position.y - robot_pose.position.y,
                                   goal_point.pose.position.x - robot_pose.position.x)
        # Convert the robot's orientation from quaternion to Euler angles
        _, _, yaw = tf_transformations.euler_from_quaternion([robot_pose.orientation.x,
                                                              robot_pose.orientation.y,
                                                              robot_pose.orientation.z,
                                                              robot_pose.orientation.w])
        # Calculate the heading error
        heading_error = self.normalize_angle(angle_to_goal - yaw)
        # Calculate the linear and angular velocities
        linear_velocity = max_linear_velocity * (1 - abs(heading_error))
        angular_velocity = max_angular_velocity * heading_error
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
    goal_pose.pose.position.x = 4.0
    node.send_goal(goal_pose)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
