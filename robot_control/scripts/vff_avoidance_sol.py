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

        if self.laser_scan != None: 

            vff_vector = self.get_vff(self.laser_scan)
            v = vff_vector['result']
            angle = math.atan2(v[1], v[0])
            module = math.sqrt((v[0]**2) + (v[1]**2))

            vel = Twist()
            vel.linear.x = max(0.0, min(module, 0.3))
            vel.angular.z = max(-0.5, min(angle, 0.5))

            self.vel_pub.publish(vel)

            marker = self.get_debug_vff(vff_vector)
            self.get_logger().info(f"linear x : >> {vel.linear.x}")
            self.get_logger().info(f"angular z : >> {vel.angular.z}")
            self.marker_pub.publish(marker)
    
    # Calculate the Virtual Force Field based on laser scan data
    def get_vff(self, scan):
        OBSTACLE_DISTANCE = 2.00  # Threshold distance for obstacle influence
        # Initialize the VFF vectors
        vff_vector = {'attractive': [0.5, 0.0],  # Goal-directed vector
                      'repulsive': [0.0, 0.0],  # Obstacle avoidance vector
                      'result': [0.0, 0.0]}  # Combined vector
        
        min_idx = np.argmin(scan.ranges)
        distance_min = scan.ranges[min_idx]
        
        # Find the nearest obstacle
        # If the nearest obstacle is within the influence threshold, calculate the repulsive vector
        if distance_min < OBSTACLE_DISTANCE:
            angle = scan.angle_min + (scan.angle_increment * min_idx)
            opposite_angle = angle + math.pi
            complementary_dist  = OBSTACLE_DISTANCE - distance_min
            vff_vector['repulsive'][0] = math.cos(opposite_angle) * complementary_dist  
            vff_vector['repulsive'][1] = math.sin(opposite_angle) * complementary_dist  

        
        vff_vector['result'][0] = vff_vector['attractive'][0] + vff_vector['repulsive'][0]  
        vff_vector['result'][1] = vff_vector['attractive'][1] + vff_vector['repulsive'][1]         
       
        # Calculate the resultant vector by combining attractive and repulsive vectors
        return vff_vector

# Main function to execute the node
def main(args=None):
    rclpy.init(args=args)
    node = VFF_Avoidance()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()