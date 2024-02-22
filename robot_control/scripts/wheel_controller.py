#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class WheelController(Node):
    def __init__(self):
        super().__init__('wheel_controller')
        self.wheel_pub = self.create_publisher(Float64MultiArray, '/velocity_controllers/commands', 10)
        self.create_timer(0.05, self.robot_timer)

    def robot_timer(self):
        self.get_logger().info(">>here")
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [3.0,5.0] # left, right
        self.wheel_pub.publish(wheel_msg)

# Main function to execute the node
def main(args=None):
    rclpy.init(args=args)
    node = WheelController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


        # def get_robot_pose(self):
        # try:
        #     rwheel = self.tf_buffer.lookup_transform('base_link', 'right_wheel', rclpy.time.Time())
        #     lwheel = self.tf_buffer.lookup_transform('base_link', 'left_wheel', rclpy.time.Time())
        #     # trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

        #     rwheel_pose = Pose()
        #     lwheel_pose = Pose()
        #     # map_pose = Pose()

        #     rwheel_pose.position.x = rwheel.transform.translation.x
        #     rwheel_pose.position.y = rwheel.transform.translation.y
        #     rwheel_pose.position.z = rwheel.transform.translation.z
        #     rwheel_pose.orientation = rwheel.transform.rotation

        #     lwheel_pose.position.x = lwheel.transform.translation.x
        #     lwheel_pose.position.y = lwheel.transform.translation.y
        #     lwheel_pose.position.z = lwheel.transform.translation.z
        #     lwheel_pose.orientation = lwheel.transform.rotation

        #     # map_pose.position.x = trans.transform.translation.x
        #     # map_pose.position.y = trans.transform.translation.y
        #     # map_pose.position.z = trans.transform.translation.z
        #     # map_pose.orientation = trans.transform.rotation
        #     self.get_logger().info(f'>> {rwheel} \n')
        #     return [lwheel_pose.orientation ,  rwheel_pose.orientation] # , map_pose
        # except (LookupException, ConnectivityException, ExtrapolationException) as e:
        #     self.get_logger().error('Could not transform from base_link to map: %s' % str(e))
        #     return None
        