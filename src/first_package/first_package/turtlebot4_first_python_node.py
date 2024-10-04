import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from irobot_create_msgs.msg import InterfaceButtons, LightringLeds
from geometry_msgs.msg import Twist

class TurtleBot4FirstNode(Node):
    lights_on_ = False

    def __init__(self):
        super().__init__('turtlebot4_first_python_node')

        self.get_logger().info('TurtleBot4FirstNode has been started.')
        self.timer_button_2 = self.create_timer(0.1, self.timer_button_2_callback)

        self.button_2_pressed = False

        # Subscribe to the /interface_buttons topic
        self.interface_buttons_subscriber = self.create_subscription(
            InterfaceButtons,
            '/turtle/interface_buttons',
            self.interface_buttons_callback,
            qos_profile_sensor_data)

        # Create a publisher for the /cmd_lightring topic
        self.lightring_publisher = self.create_publisher(
            LightringLeds,
            '/turtle/cmd_lightring',
            qos_profile_sensor_data)
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/turtle/cmd_vel',
            qos_profile_sensor_data)

    # Interface buttons subscription callback
    def interface_buttons_callback(self, create3_buttons_msg: InterfaceButtons):
        # Button 1 is pressed
        if create3_buttons_msg.button_1.is_pressed:
            self.get_logger().info('Button 1 Pressed!')
            self.button_1_function()
        if create3_buttons_msg.button_2.is_pressed:
            self.get_logger().info('Button 2 Pressed!')
            self.button_2_pressed = not self.button_2_pressed
            self.get_logger().info(f'Button 2 Pressed: {self.button_2_pressed}')
            

    # Perform a function when Button 1 is pressed
    def button_1_function(self):
        # Create a ROS 2 message
        lightring_msg = LightringLeds()
        # Stamp the message with the current time
        lightring_msg.header.stamp = self.get_clock().now().to_msg()

        # Lights are currently off
        if not self.lights_on_:
            # Override system lights
            lightring_msg.override_system = True

            # LED 0
            lightring_msg.leds[0].red = 255
            lightring_msg.leds[0].blue = 0
            lightring_msg.leds[0].green = 0

            # LED 1
            lightring_msg.leds[1].red = 0
            lightring_msg.leds[1].blue = 255
            lightring_msg.leds[1].green = 0

            # LED 2
            lightring_msg.leds[2].red = 0
            lightring_msg.leds[2].blue = 0
            lightring_msg.leds[2].green = 255

            # LED 3
            lightring_msg.leds[3].red = 255
            lightring_msg.leds[3].blue = 255
            lightring_msg.leds[3].green = 0

            # LED 4
            lightring_msg.leds[4].red = 255
            lightring_msg.leds[4].blue = 0
            lightring_msg.leds[4].green = 255

            # LED 5
            lightring_msg.leds[5].red = 0
            lightring_msg.leds[5].blue = 255
            lightring_msg.leds[5].green = 255
        # Lights are currently on
        else:
            # Disable system override. The system will take back control of the lightring.
            lightring_msg.override_system = False

        # Publish the message
        self.lightring_publisher.publish(lightring_msg)
        # Toggle the lights on status
        self.lights_on_ = not self.lights_on_

    def timer_button_2_callback(self):
        if self.button_2_pressed:
            twist = Twist()
            twist.linear.x = 0.5
        else:
            twist = Twist()
            twist.linear.x = 0.0
        self.cmd_vel_publisher.publish(twist)





def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot4FirstNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


