import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from enum import Enum

class Direction(Enum):
    LEFT = 0
    FORWARD = 270
    RIGHT = 540
    BACKWARD = 810

class TurtleBot4ScanTestNode(Node):
    def __init__(self):
        super().__init__('turtlebot4_scan_test_node')
        self.get_logger().info('TurtleBot4ScanTestNode has been started.')
        
        self.obstacle_detected = False

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/turtle/scan',
            self.scan_callback,
            10)
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/turtle/cmd_vel',
            10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)

    def scan_callback(self, msg):
        print(self.obstacle_detected)
        if msg.ranges[Direction.FORWARD.value] < 0.5:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def timer_callback(self):
        twist = Twist()
        if self.obstacle_detected:
            twist.linear.x = 0.0
 
        else:
            twist.linear.x = 0.5

        self.cmd_vel_publisher.publish(twist)
        

    

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot4ScanTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

