#!/usr/bin/python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose, TransformStamped, Twist, PoseStamped
from sensor_msgs.msg import JointState, LaserScan
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
        self.prev_time_test = None 

        # < Timer - 
        
        # > Constant -
        self.WHEEL_RADIUS = 0.075
        self.WHEEL_SEPARATION = 0.4
        # < Constant - 

        # > Global Variable - 
        self.wheel_omega = [0.0, 0.0]
        self.robot_twist = [0.0, 0.0]
        self.robot_position = [0.0, 0.0, 0.0] # x, y, theta
        self.target_goal = [-3,1] #[1,0]
        # < Global Variable

        # > Subscription / Publisher -
        self.wheel_pub = self.create_publisher(Float64MultiArray, '/velocity_controllers/commands', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10) # Read Velocity
        self.create_subscription(Path, 'robot_path', self.get_path_callback, 10) # Read Velocity
        self.create_subscription(LaserScan, "/scan", self.laser_scan_callback, 10)
        # < Subscription / Publisher -

        # > Global Planner - 
        
        # < Global Planner -

        # > TF -
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.publish_transform = TransformBroadcaster(self)

        # < TF - 
        self.path = None
        self.current_pose_index = 0
        self.total_path = None
        self.laser_scan = None
        self.robot_init = True
        self.get_path = False
        self.sent_linear_velocity, self.sent_angular_velocity = 0,0
        self.lookahead_distance = 1.00 # 0.40
        self.goal_threshold = 0.15
        self.reach_flag = True


    def laser_scan_callback(self, msg):
        self.laser_scan = msg

    def get_path_callback(self, msg): 
        self.get_path = True

        self.path = msg.poses
        self.total_path = len(self.path)
        self.prev_time_test = self.get_clock().now()

    def joint_state_callback(self, msg):
        self.wheel_omega = msg.velocity

    def robot_timer(self)->None:

        self.calculate_wheel_odometry()
        self.pub_transform()
        self.robot_twist = self.forward_kinematics(self.wheel_omega[1], self.wheel_omega[0]) # right | left
        if (self.robot_init):
            self.sent_linear_velocity, self.sent_angular_velocity = self.goal_to_goal_controller()
        if (self.get_path):
            self.sent_linear_velocity, self.sent_angular_velocity = self.pure_pursuit_controller()
            # self.get_logger().info(f"linear {self.sent_linear_velocity} | angular : {self.sent_angular_velocity}")
        wheel_omega = self.inverse_kinematics(self.sent_linear_velocity, self.sent_angular_velocity)
        self.pub_wheel_velocity(wheel_omega[0], wheel_omega[1])
            

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
            self.target_goal = [0,0]
            self.robot_init = False
            


        print(f'dx : {dp_x} | dy : {dp_x}')
        return linear_velocity, angular_velocity 

    # < Basic Controller --

    # > Purepursuit + VFF --

    def pure_pursuit_controller(self):
        linear_velocity, angular_velocity = 0,0
        if (self.laser_scan != None):
            target_index = self.calculate_goal_point(self.path, self.current_pose_index)
            vff_vector = self.get_vff(self.laser_scan, target_index)
            linear_velocity, angular_velocity = self.calculate_velocities(vff_vector['result']) 
        return linear_velocity, angular_velocity

    def calculate_goal_point(self, path, start_index):
        temp_pose_index = start_index
        target_point = path[temp_pose_index].pose.position
        curr_dist = self.distance_between_points(target_point, [self.robot_position[0],self.robot_position[1]])
        while curr_dist < self.lookahead_distance and temp_pose_index  < (self.total_path - 1):
            temp_pose_index += 1
            target_point = path[temp_pose_index].pose.position
            curr_dist = self.distance_between_points(target_point, [self.robot_position[0],self.robot_position[1]])

        self.current_pose_index = temp_pose_index
        return self.current_pose_index

    def calculate_velocities(self, target):
        max_linear_velocity = 0.40  # Maximum linear velocity
        max_angular_velocity = 1.00  # Maximum angular velocity
        linear_velocity, angular_velocity = 0,0
        target_x = target[0]
        target_y = target[1]
        dp_x = target_x
        dp_y = target_y 

        theta = self.robot_position[2]
        e = np.arctan2(dp_y , dp_x ) - theta
        K = 0.70
        w = K * np.arctan2(np.sin(e), np.cos(e))

        if np.linalg.norm([dp_x, dp_y]) > self.goal_threshold:
            linear_velocity = max_linear_velocity
            angular_velocity = w
            if angular_velocity > max_angular_velocity:
                angular_velocity = max_angular_velocity
                linear_velocity = 0.0
        else:
            linear_velocity = 0.0
            angular_velocity = 0.0
            if (self.reach_flag):
                use_time = (self.get_clock().now().nanoseconds - self.prev_time_test.nanoseconds )/ S_TO_NS
                self.get_logger().info(f'Reach Goal Time : {use_time}') 
                self.reach_flag = False

        return linear_velocity, angular_velocity

    def distance_between_points(self, point1, robot_pos):
        return math.sqrt((point1.x - robot_pos[0])**2 + (point1.y - robot_pos[1])**2)

    def get_vff(self, scan, target_index):
        OBSTACLE_DISTANCE = 1.00 #0.60  # Threshold distance for obstacle influence
        target_x = self.path[target_index].pose.position.x - self.robot_position[0]
        target_y = self.path[target_index].pose.position.y - self.robot_position[1]


        vff_vector = {'attractive': [target_x , target_y ],  # Goal-directed vector
                      'repulsive': [0.0, 0.0],  # Obstacle avoidance vector
                      'result': [0.0, 0.0]}  # Combined vector
        
        min_idx = np.argmin(scan.ranges)
        distance_min = scan.ranges[min_idx]
        
        if distance_min < OBSTACLE_DISTANCE:
            angle = scan.angle_min + (scan.angle_increment * min_idx)
            opposite_angle = angle + math.pi
            complementary_dist  = OBSTACLE_DISTANCE - distance_min
            gain_replusive = 5.00
            power_replusive = 2.00
            vff_vector['repulsive'][0] = math.cos(opposite_angle) * ((complementary_dist * gain_replusive ) ** power_replusive)
            vff_vector['repulsive'][1] = math.sin(opposite_angle) * ((complementary_dist * gain_replusive ) ** power_replusive)

        
        vff_vector['result'][0] = vff_vector['attractive'][0] + vff_vector['repulsive'][0]  
        vff_vector['result'][1] = vff_vector['attractive'][1] + vff_vector['repulsive'][1]   
        if (~self.reach_flag and np.linalg.norm(vff_vector['result']) < 0.1 ):
            vff_vector['result'] *= 10

        self.get_logger().info(f'---- {vff_vector}')
        return vff_vector

    # < Purepursuit + VFF --

def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
