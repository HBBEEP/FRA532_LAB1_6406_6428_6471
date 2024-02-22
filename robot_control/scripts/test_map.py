#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TestMap(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.subscription = self.create_subscription(String,'map_ok',self.listener_callback,10)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

# Main function to execute the node
def main(args=None):
    rclpy.init(args=args)
    node = TestMap()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()