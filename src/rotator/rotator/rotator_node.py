#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty


def get_key():
    fd = sys.stdin.fileno()
    old_attr = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)
    return key

class Rotator_node(Node):
    def __init__(self):
        super().__init__('Rotator_node')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed = 0.0   
        self.turn = 0.5    
        self.get_logger().info(f"Speed: {self.speed}, Turn: {self.turn}")
        self.get_logger().info("Keys: a=left, d=right, x=stop, Ctrl+C=exit")

    def send_cmd(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.pub.publish(msg)

    def loop(self):
        try:
            while rclpy.ok():
                key = get_key()
                if key == 'a':  
                    self.send_cmd(0.0, self.turn)
                elif key == 'd':  
                    self.send_cmd(0.0, -self.turn)
                elif key == 'x':  
                    self.send_cmd(0.0, 0.0)
                    self.get_logger().info("Stop")
                elif key == '\x03':     # Ctrl+C
                    break
                rclpy.spin_once(self, timeout_sec=0.0)
        except KeyboardInterrupt:
            pass
        finally:
            self.send_cmd(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = Rotator_node()
    node.loop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
