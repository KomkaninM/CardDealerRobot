import math
import os
import sys
import termios

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool  # [ADDED] Bool import
from nav_msgs.msg import Odometry
import numpy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

ros_distro = os.environ.get('ROS_DISTRO', 'humble').lower()
if ros_distro == 'humble':
    from geometry_msgs.msg import Twist as CmdVelMsg
else:
    from geometry_msgs.msg import TwistStamped as CmdVelMsg

terminal_msg = """
Relative Move
theta: goal orientation (range: -180 ~ 180, unit: deg)
"""
class Turtlebot3Path():
    @staticmethod
    def turn(angle, angular_velocity, step):
        twist = CmdVelMsg()
        angle = math.atan2(math.sin(angle), math.cos(angle))
        threshold_rad = math.radians(2)
        if abs(angle) > threshold_rad:
            twist.angular.z = angular_velocity if angle > 0 else -angular_velocity
        else:
            step += 1
            twist.angular.z = 0.0
        return twist, step

class rotator_node(Node):
    def __init__(self):
        super().__init__('rotator_node')
        self.odom = Odometry() 

        self.has_new_angle = False
        self.angle_input = 0.0
        self.last_pose_theta = 0.0
        self.goal_pose_theta = 0.0
        self.step = 1
        self.get_key_state = False
        self.init_odom_state = False
        qos = QoSProfile(depth=10)

        self.cmd_vel_pub = self.create_publisher(CmdVelMsg, 'cmd_vel', qos)
        self.angle_pub = self.create_publisher(Float32, '/rotator/angle_now', 10)
        
        # [ADDED] Publisher for status
        self.status_pub = self.create_publisher(Bool, '/rotator/status', 10)

        self.sub = self.create_subscription(Float32, '/rotator/angle', self.angle_cb, 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos)
        self.update_timer = self.create_timer(0.010, self.update_callback)
        self.get_logger().info('TurtleBot3 Rotator node has been initialised.')
        self.get_logger().info("Published to: /rotator/status | Subscribed to: /rotator/angle")

    def angle_cb(self, msg):
        self.angle_input = math.radians(msg.data)
        self.has_new_angle = True
        self.get_logger().info(f"Received angle: {self.angle_input}")

    def odom_callback(self, msg):
        _, _, self.last_pose_theta = self.euler_from_quaternion(msg.pose.pose.orientation)
        msg = Float32()
        msg.data = math.degrees(self.last_pose_theta)
        self.angle_pub.publish(msg)
        self.init_odom_state = True

    def update_callback(self):
        if self.init_odom_state:
            self.generate_path()

    def generate_path(self):
        twist = CmdVelMsg()
        if not self.init_odom_state:
            return
        
        # Check if we have a new angle to start processing
        if self.has_new_angle and not self.get_key_state:
            self.goal_pose_theta = self.angle_input
            self.get_key_state = True
            self.has_new_angle = False
            
            # Optional: Publish False when starting to indicate "Busy"
            # busy_msg = Bool()
            # busy_msg.data = False
            # self.status_pub.publish(busy_msg)
        
        # Use 'elif' to only enter this block if we are actively moving
        elif self.get_key_state:
            if self.step == 1:
                angle = self.goal_pose_theta - self.last_pose_theta

                self.get_logger().info(
                    f"current_theta={math.degrees(self.last_pose_theta):.2f} deg | "
                    f"goal_theta={math.degrees(self.goal_pose_theta):.2f} deg | "
                    f"error={math.degrees(angle):.2f} deg"
                )
                angular_velocity = 0.8
                twist, self.step = Turtlebot3Path.turn(angle, angular_velocity, self.step)

            elif self.step == 2:
                twist.angular.z = 0.0
                self.step = 1
                self.get_key_state = False # This turns off the 'elif' for the next loop
                
                # [ADDED] Publish True when destination is reached
                status_msg = Bool()
                status_msg.data = True
                self.status_pub.publish(status_msg)
                self.get_logger().info("Target Reached. Published /rotator/status = True")

            if ros_distro == 'humble':
                self.cmd_vel_pub.publish(twist)
            else:
                stamped = CmdVelMsg()
                stamped.header.stamp = self.get_clock().now().to_msg()
                stamped.twist = twist
                self.cmd_vel_pub.publish(stamped)
                
    def euler_from_quaternion(self, quat):
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = numpy.arctan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (w * y - z * x)
        pitch = numpy.arcsin(sinp)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = numpy.arctan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    node = rotator_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_twist = CmdVelMsg()
        node.cmd_vel_pub.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()