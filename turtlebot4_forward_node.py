import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from irobot_create_msgs.msg import InterfaceButtons, LightringLeds
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

class TurtleBot4ForwardNode(Node):
    def __init__(self):
        super().__init__('turtlebot4_forward_node')
        self.get_logger().info('TurtleBot4ForwardNode has been started.')


        # Publish to the /cmd_vel topic
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            QoSProfile(depth=10))
        self.timer = self.create_timer(0.1, self.timer_callback)


    def timer_callback(self):
        twist = Twist()
        twist.linear.x = 0.1
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info('Publishing forward command')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot4ForwardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    