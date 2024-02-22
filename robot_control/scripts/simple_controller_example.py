#!/usr/bin/python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import tf2_ros
import tf_transformations
import math

# Define a Controller class that extends the Node class from ROS 2
class Controller(Node):
    def __init__(self):
        # Initialize the node with the name 'controller'
        super().__init__('controller')
        # Create a publisher for velocity commands
        self.command_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # Set the timer period for periodic callback execution
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Initialize pose and goal
        self.pose = Pose()
        self.goal = np.array([5.0, 0.0])
        # Initialize TF2 buffer and listener for pose transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.angular = False

    # Callback method for the timer
    def timer_callback(self):
        try:
            # Attempt to get the current transform from the map to the base_link
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            pose = Pose()
            pose.position.x = trans.transform.translation.x
            pose.position.y = trans.transform.translation.y
            pose.position.z = trans.transform.translation.z
            pose.orientation = trans.transform.rotation
            self.pose = pose
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            # Log an error if the transformation is not possible
            self.get_logger().error('Could not transform from base_link to map: %s' % str(e))
            return None
        msg = self.control()
        self.command_publisher.publish(msg)
        

    # Callback method for the pose subscription
    def pose_callback(self, msg):
        self.pose = msg


    # Method to calculate the control commands
    # def control(self):
    #     KP = 0.50
    #     THRESHOLD = 0.3
    #     THRESHOLD_W = 0.1



    #     diff_pos_x = self.goal[0] - self.pose.position.x
    #     diff_pos_y = self.goal[1] - self.pose.position.y

    #     _, _, theta = tf_transformations.euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y,self.pose.orientation.z, self.pose.orientation.w])
    #     alpha = np.arctan2(diff_pos_y, diff_pos_x)
    #     e_orient = alpha - theta
        
    #     robo_angular = KP * np.arctan2(np.sin(e_orient), np.cos(e_orient))
        


    #     if np.linalg.norm([diff_pos_x, diff_pos_y]) > THRESHOLD :
    #         robo_velo = 0.5
    #     else:
    #         robo_velo =  0
    #         robo_angular = 0

    #     if  robo_angular <= THRESHOLD_W:
    #         self.angular = True

    #     if self.angular:
    #         robo_angular = 0


    #     msg = Twist()
    #     msg.linear.x = robo_velo
    #     msg.angular.z = robo_angular
    #     self.get_logger().info(f'-----{robo_angular}-------------------')

    #     return msg
        # Method to calculate the control commands
    def control(self):
        # Initialize a Twist message
        msg = Twist()
        # Calculate the difference between the current position and the goal
        current_position = np.array([self.pose.position.x, self.pose.position.y])
        dp = self.goal - current_position
        # Convert the quaternion to Euler angles to get the robot's orientation
        _, _, theta = tf_transformations.euler_from_quaternion([self.pose.orientation.x,
                                                               self.pose.orientation.y,
                                                               self.pose.orientation.z,
                                                               self.pose.orientation.w])
        # Calculate the error in orientation
        e = np.arctan2(dp[1], dp[0]) - theta
        # Control gain
        K = 0.50
        # Calculate the angular velocity
        w = K * np.arctan2(np.sin(e), np.cos(e))
        # Print the current position for debugging
        print(current_position, theta)
        # Set the linear velocity based on the distance to the goal
        if np.linalg.norm(dp) > 0.3:
            v = 0.50
        else:
            v = 0.0
            w = 0.0


        # Assign the computed velocities to the message
        msg.linear.x = v
        msg.angular.z = w
        return msg

        

# Main function to initialize and run the ROS 2 node
def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()