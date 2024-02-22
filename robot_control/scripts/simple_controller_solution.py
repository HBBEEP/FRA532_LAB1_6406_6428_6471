import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import tf2_ros
import tf_transformations

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
        # Subscribe to the pose topic to get the robot's current position
        self.pose_subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        # Initialize pose and goal
        self.pose = Pose()
        self.goal = np.array([0.0, 0.0])
        # Initialize TF2 buffer and listener for pose transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

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
        
        # Compute control commands and publish them
        msg = self.control()
        self.command_publisher.publish(msg)
    
    # Callback method for the pose subscription
    def pose_callback(self, msg):
        # Update the robot's current pose
        self.pose = msg

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
        print(current_position)
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