import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class TeleopTwistJoy(Node):
    def __init__(self):
        super().__init__('Teleop_Keymapping_node')
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joyCallback, 10)
        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.max_linear_speed = 4
        self.max_angular_speed = 10


    def joyCallback(self, msg):
        self.sendCommand(msg)

    def sendCommand(self, msg):
        t = Twist()
        # safety lock, press top left button
        if msg.buttons[4] == 1.0:
            t.linear.x = msg.axes[1]
            # use to represent angular velocity
            t.angular.z = msg.axes[3]
            self.twist_pub.publish(t)
        else:
            t.linear.x = 0.0
            t.angular.z = 0.0
            self.twist_pub.publish(t)

def main(args = None):
    rclpy.init(args=args)
    ttj = TeleopTwistJoy()
    rclpy.spin(ttj)
    rclpy.shutdown()


if __name__ == '__main__':
    main()