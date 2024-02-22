#!/usr/bin/python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose, TransformStamped, Twist, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException, TransformBroadcaster
import numpy as np
import math
import tf_transformations
from rclpy.constants import S_TO_NS

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')

        # > Timer -
        self.TIME_STEP = 0.05
        self.create_timer(self.TIME_STEP, self.robot_timer)
        self.prev_time = self.get_clock().now()
        self.prev_time2 = self.get_clock().now()

        # < Timer - 
        
        # > Constant -
        self.WHEEL_RADIUS = 0.075
        self.WHEEL_SEPARATION = 0.4
        # < Constant - 

        # > Global Variable - 
        self.wheel_omega = [0.0, 0.0]
        self.robot_twist = [0.0, 0.0]
        self.robot_position = [0.0, 0.0, 0.0] # x, y, theta
        self.target_goal = [2, 2] #[-3,1]
        # < Global Variable



        # > Subscription -
        self.wheel_pub = self.create_publisher(Float64MultiArray, '/velocity_controllers/commands', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10) # Read Velocity
        self.create_subscription(Path, 'robot_path', self.get_path_callback, 10) # Read Velocity

        # < Subscription -

        # > Global Planner - 
        self.action_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.path = None
        self.total_path = None
        self.get_path = True
        # < Global Planner -
        self.publisher_ = self.create_publisher(String, 'map_ok', 10)
        self.run_controller = False

        # > TF -
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.publish_transform = TransformBroadcaster(self)
        # self.calculate_wheel_odometry()
        # self.pub_transform()

        # < TF - 
        self.goal_marker_publisher = self.create_publisher(PoseStamped, 'goal_marker', 10)

    def joint_state_callback(self, msg):
        self.wheel_omega = msg.velocity

    def robot_timer(self)->None:
        # ok
        self.calculate_wheel_odometry()
        self.pub_transform()
        # self.publish_odometry()
        # self.robot_twist = self.forward_kinematics(self.wheel_omega[1], self.wheel_omega[0]) # right | left
        # linear_velocity, angular_velocity = self.goal_to_goal_controller()
        # wheel_omega = self.inverse_kinematics(linear_velocity, angular_velocity)
        # self.pub_wheel_velocity(wheel_omega[0], wheel_omega[1])
        self.get_logger().info(f'----- {self.robot_position[0]}    {self.robot_position[1]}-----')

        # ok

    def get_path_callback(self, msg): 
        self.get_logger().info(f'----- {msg}')
        self.path = msg

    def calculate_omega(self, rwheel:float, lwheel:float):
        return [lwheel / self.TIME_STEP , rwheel/self.TIME_STEP]

    def calculate_wheel_odometry(self)->None:
        dt = self.get_clock().now() - self.prev_time
        dx = self.robot_twist[0] * math.cos(self.robot_position[2] ) * (dt.nanoseconds / S_TO_NS)
        dy = self.robot_twist[0] * math.sin(self.robot_position[2] ) * (dt.nanoseconds / S_TO_NS)

        dtheta = self.robot_twist[1] * (dt.nanoseconds / S_TO_NS)
        self.robot_position[0] += dx
        self.robot_position[1] += dy
        self.robot_position[2] += dtheta
        self.robot_position[2] = math.atan2(math.sin(self.robot_position[2]), math.cos(self.robot_position[2]))
        self.prev_time = self.get_clock().now()

    def pub_transform(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.robot_position[0]
        t.transform.translation.y = self.robot_position[1]

        q = tf_transformations.quaternion_from_euler(0.0, 0.0, self.robot_position[2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.publish_transform.sendTransform(t)


    def publish_odometry(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.robot_position[0]
        msg.pose.pose.position.y = self.robot_position[1]

        q = tf_transformations.quaternion_from_euler(0.0, 0.0, self.robot_position[2])
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        self.odom_pub.publish(msg)

    def pub_wheel_velocity(self, right_wheel:float, left_wheel:float)->None:
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [left_wheel, right_wheel]
        self.wheel_pub.publish(wheel_msg)

    def forward_kinematics(self, rwheel_velocity, lwheel_velocity):
        fk_constant = self.WHEEL_RADIUS * np.array([[0.5,0.5],
                                [1/self.WHEEL_SEPARATION, -1/self.WHEEL_SEPARATION]])
        wheel_velocity = np.array([rwheel_velocity, lwheel_velocity])
        robot_twist = np.dot(fk_constant, wheel_velocity)
        robot_twist[0] = 0 if abs(robot_twist[0]) < 0.0001 else robot_twist[0] 
        robot_twist[1] = 0 if abs(robot_twist[1]) < 0.0001 else robot_twist[1] 
        return robot_twist

    def inverse_kinematics(self, linear_velocity:float, angular_velocity ):
        ik_constant = (1/self.WHEEL_RADIUS)* np.array([[1, 0.5 * self.WHEEL_SEPARATION],
                                [1 , -0.5 *self.WHEEL_SEPARATION]])
        robot_twist = np.array([linear_velocity, angular_velocity])
        wheel_velocity = np.dot(ik_constant, robot_twist)

        return wheel_velocity
    
    # > Basic Controller --
    def goal_to_goal_controller(self):
        dp_x = self.target_goal[0] - self.robot_position[0] 
        dp_y = self.target_goal[1] - self.robot_position[1] 
        theta = self.robot_position[2]
        e = np.arctan2(dp_y , dp_x) - theta

        K = 0.60
        angular_velocity = K * np.arctan2(np.sin(e), np.cos(e))
        if np.linalg.norm([dp_x, dp_y]) > 0.1:
            linear_velocity = 0.50
        else:
            linear_velocity = 0.0
            angular_velocity = 0.0
        print(f'dx : {dp_x} | dy : {dp_x}')
        return linear_velocity, angular_velocity 

    # < Basic Controller --

    # > Global Planner --
    def goal_init(self):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = 2.0
        goal_pose.pose.position.y = -3.0
        self.send_goal(goal_pose)

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
        self.total_path = len(self.path)


    # < Global Planner --
    
    def twist_manager(self, vector_x, vector_y):
        angle = math.atan2(vector_x[1], vector_y[0])
        module = math.sqrt((vector_x[0]**2) + (vector_y[1]**2))

        vel = Twist()
        vel.linear.x = max(0.0, min(module, 0.3))
        vel.angular.z = max(-0.5, min(angle, 0.5))
        return vel
    # purepursuit + vff

    def get_vff(self, scan, target_index):
        OBSTACLE_DISTANCE = 1.00  # Threshold distance for obstacle influence
        target_x = self.path[target_index].pose.position.x 
        target_y = self.path[target_index].pose.position.y 


        vff_vector = {'attractive': [target_x , target_y ],  # Goal-directed vector
                      'repulsive': [0.0, 0.0],  # Obstacle avoidance vector
                      'result': [0.0, 0.0]}  # Combined vector
        
        min_idx = np.argmin(scan.ranges)
        distance_min = scan.ranges[min_idx]
        
        if distance_min < OBSTACLE_DISTANCE:
            angle = scan.angle_min + (scan.angle_increment * min_idx)
            opposite_angle = angle + math.pi
            complementary_dist  = OBSTACLE_DISTANCE - distance_min
            vff_vector['repulsive'][0] = math.cos(opposite_angle) * complementary_dist  
            vff_vector['repulsive'][1] = math.sin(opposite_angle) * complementary_dist  

        
        vff_vector['result'][0] = vff_vector['attractive'][0] + vff_vector['repulsive'][0]  
        vff_vector['result'][1] = vff_vector['attractive'][1] + vff_vector['repulsive'][1]         
       
        return vff_vector

    # ---


# Main function to execute the node
def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()
    node.goal_init()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
