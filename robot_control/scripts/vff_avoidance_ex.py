#!/usr/bin/python3
# Import necessary ROS 2 and other Python libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import math
import numpy as np

# Define a class for implementing VFF Avoidance using ROS 2
class VFF_Avoidance(Node):
    # Constructor of the class
    def __init__(self):
        # Initialize the ROS 2 node with a name
        super().__init__('vff_avoidance')
        # Create a subscription to the laser scan topic
        self.create_subscription(LaserScan, "/scan", self.laser_scan_callback, 10)
        # Create a timer to regularly invoke the VFF controller logic
        self.create_timer(0.05, self.vff_controller)
        # Publisher for robot velocity commands
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        # Publisher for visualization markers
        self.marker_pub = self.create_publisher(MarkerArray, "marker_debug", 10)
        # Variable to store the latest laser scan data
        self.laser_scan = None
        # Constants for visualization colors
        self.RED = 0
        self.GREEN = 1
        self.BLUE = 2

        self.run = 1

    # Callback for processing laser scan messages
    def laser_scan_callback(self, msg):
        # Store the laser scan data for use in the controller
        self.laser_scan = msg
    
    # Generate visualization markers for the VFF vectors
    def get_debug_vff(self, vff_vectors):
        marker_array = MarkerArray()
        # Create and add markers for attractive, repulsive, and resultant vectors
        marker_array.markers.append(self.make_marker(vff_vectors['attractive'], self.BLUE))
        marker_array.markers.append(self.make_marker(vff_vectors['repulsive'], self.RED))
        marker_array.markers.append(self.make_marker(vff_vectors['result'], self.GREEN))
        return marker_array

    # Utility function to create a marker for visualization
    def make_marker(self, vector, vff_color):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        # Define the start and end points of the marker
        start = Point(x=0.0, y=0.0, z=0.0)
        end = Point(x=vector[0], y=vector[1], z=0.0)
        marker.points = [start, end]
        # Set the scale of the marker
        marker.scale.x = 0.05
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        # Set the color of the marker based on the type of vector
        color = ColorRGBA(a=1.0)  # Fully opaque
        if vff_color == self.RED:
            marker.id = 0
            color.r = 1.0
        elif vff_color == self.GREEN:
            marker.id = 1
            color.g = 1.0
        elif vff_color == self.BLUE:
            marker.id = 2
            color.b = 1.0
        marker.color = color
        return marker

    # The main controller logic for VFF-based obstacle avoidance
    def vff_controller(self):
        # Only proceed if laser scan data is available
        msg = Twist()

        if self.laser_scan != None and self.run == 1: 
            # self.run = 2
            # self.get_logger().info(f'>>{self.laser_scan}')
            # self.get_logger().info(f'>>{min(self.laser_scan.ranges)}')
            # self.get_logger().info(f'>>{self.laser_scan.angle_increment}')
            # self.get_logger().info(f'>>{vff_vector}')

            vff_vector = self.get_vff(self.laser_scan)


            w = np.arctan2(np.sin(vff_vector[1]), np.cos(vff_vector[0]))
            msg.linear.x = np.cos(vff_vector[0])
            msg.angular.z = 0.5 * w

            # Calculate the VFF based on the current laser scan
            # Extract the resultant vector for calculating velocity commands
            # Create the velocity command message
            # Publish the velocity command
            # Publish visualization markers
            self.get_logger().info(f'>>{msg}')
            self.vel_pub.publish(msg)

    
    # Calculate the Virtual Force Field based on laser scan data
    def get_vff(self, scan):
        OBSTACLE_DISTANCE = 2.00  # Threshold distance for obstacle influence
        # Initialize the VFF vectors
        vff_vector = {'attractive': [0.5, 0.0],  # Goal-directed vector
                      'repulsive': [0.0, 0.0],  # Obstacle avoidance vector
                      'result': [0.0, 0.0]}  # Combined vector
        
        min_idx = np.argmin(scan.ranges)
        distance_min = scan.ranges[min_idx]
        
        theta = (-3.14 + (self.laser_scan.angle_increment * min_idx))
        repulsive_x = - distance_min  * math.cos(theta)
        repulsive_y = - distance_min  * math.sin(theta)
        # Find the nearest obstacle
        # If the nearest obstacle is within the influence threshold, calculate the repulsive vector
        if distance_min < OBSTACLE_DISTANCE:
            # Opposite direction to the obstacle
            # Convert to Cartesian coordinates
            vff_vector['repulsive'] = [repulsive_x,repulsive_y]
        
        vff_vector['result'][0] = vff_vector['attractive'][0] + vff_vector['repulsive'][0]  
        vff_vector['result'][1] = vff_vector['attractive'][1] + vff_vector['repulsive'][1]         
       
        # Calculate the resultant vector by combining attractive and repulsive vectors
        return vff_vector['result']

# Main function to execute the node
def main(args=None):
    rclpy.init(args=args)
    node = VFF_Avoidance()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()