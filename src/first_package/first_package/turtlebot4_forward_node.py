import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
import math

class TurtleBot4ForwardNode(Node):
    def __init__(self):
        super().__init__('turtlebot4_forward_node')
        self.get_logger().info('TurtleBot4ForwardNode has been started.')

        # Publish to the /cmd_vel topic
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/turtle/cmd_vel',
            QoSProfile(depth=10))
        
        # Subscribe to the /odom topic
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/turtle/odom',
            self.odom_callback,
            QoSProfile(depth=10))
        
        self.forward_timer = self.create_timer(0.1, self.forward_timer_callback)
        self.start_time = self.get_clock().now()
        self.is_turning = False
        self.initial_yaw = None
        self.target_yaw = None

    def forward_timer_callback(self):
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9  # Convert to seconds

        twist = Twist()
        if elapsed_time < 3.0:
            twist.linear.x = 0.1
        else:
            twist.linear.x = 0.0
            self.forward_timer.cancel()  # Stop the forward timer
            self.is_turning = True

        self.cmd_vel_publisher.publish(twist)

    def odom_callback(self, msg):
        if self.is_turning:
            orientation_q = msg.pose.pose.orientation
            yaw = self.quaternion_to_yaw(orientation_q)

            if self.initial_yaw is None:
                self.initial_yaw = yaw
                self.target_yaw = self.initial_yaw + math.pi / 2  # 90 degrees in radians
                self.get_logger().info(f'Initial yaw: {self.initial_yaw}, Target yaw: {self.target_yaw}')

            twist = Twist()
            if abs(yaw - self.target_yaw) > 0.01:  # Allow some tolerance
                twist.angular.z = 0.5  # Adjust this value to control the turning speed
                self.get_logger().info(f'Turning... Current yaw: {yaw}')
            else:
                twist.angular.z = 0.0
                self.is_turning = False  # Stop turning
                self.get_logger().info('Finished turning')

            self.cmd_vel_publisher.publish(twist)

    def quaternion_to_yaw(self, q):
        # Convert quaternion to yaw angle
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot4ForwardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()