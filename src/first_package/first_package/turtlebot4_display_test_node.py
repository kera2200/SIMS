import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from turtlebot4_msgs.msg import UserDisplay

class TurtleBot4DisplayTestNode(Node):
    def __init__(self):
        super().__init__('turtlebot4_display_test_node')
        self.get_logger().info('TurtleBot4DisplayTestNode has been started.')
        
        self.display_publisher = self.create_publisher(
            UserDisplay,
            '/turtle/hmi/display',
            QoSProfile(depth=10))

        self.timer_display = self.create_timer(0.1, self.publish_display_message)

    def publish_display_message(self):
        msg = UserDisplay()
        msg.ip = 'Hello, TurtleBot4!'
        self.display_publisher.publish(msg)
        self.get_logger().info('Display message has been published.')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot4DisplayTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()