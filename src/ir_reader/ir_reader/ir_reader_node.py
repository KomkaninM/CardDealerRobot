#!/usr/bin/env python3
"""
IR Reader Node
- Read IR sensor via GPIO (BCM17)
- Subscribe /odom
- Publish:
    /ir_reader/player_detection (std_msgs/Bool)
    /ir_reader/player_angle     (std_msgs/Float32)
"""

import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

# ===== GPIO =====
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except Exception:
    GPIO_AVAILABLE = False


def quaternion_to_yaw(x, y, z, w) -> float:
    """Convert quaternion to yaw (rad)"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class IRReaderNode(Node):
    def __init__(self):
        super().__init__('ir_reader_node')

        # ---------- Parameters ----------
        self.declare_parameter('ir_gpio_bcm', 17)
        self.declare_parameter('active_low', True)
        self.declare_parameter('rate_hz', 10.0)

        self.ir_gpio = int(self.get_parameter('ir_gpio_bcm').value)
        self.active_low = bool(self.get_parameter('active_low').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)

        # ---------- Internal state ----------
        self.current_yaw = 0.0
        self.last_detected = None

        # ---------- ROS interfaces ----------
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.pub_detect = self.create_publisher(
            Bool,
            '/ir_reader/player_detection',
            10
        )

        self.pub_angle = self.create_publisher(
            Float32,
            '/ir_reader/player_angle',
            10
        )

        # ---------- GPIO setup ----------
        if GPIO_AVAILABLE:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(
                self.ir_gpio,
                GPIO.IN,
                pull_up_down=GPIO.PUD_UP   # เหมาะกับ IR active-low
            )
        else:
            self.get_logger().warning(
                'RPi.GPIO not available (not running on Raspberry Pi?)'
            )

        # ---------- Timer ----------
        period = 1.0 / max(self.rate_hz, 1.0)
        self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f'IR Reader started | GPIO={self.ir_gpio} | active_low={self.active_low}'
        )

    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.current_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def read_ir(self) -> bool:
        """Return True if IR detected"""
        if not GPIO_AVAILABLE:
            return False

        raw = GPIO.input(self.ir_gpio)  # 0 or 1
        return (raw == 0) if self.active_low else (raw == 1)

    def timer_callback(self):
        detected = self.read_ir()

        # publish only when state changes
        if detected != self.last_detected:
            self.last_detected = detected

            msg_detect = Bool()
            msg_detect.data = detected
            self.pub_detect.publish(msg_detect)

            msg_angle = Float32()
            msg_angle.data = float(self.current_yaw)
            self.pub_angle.publish(msg_angle)

            self.get_logger().info(
                f'IR {"Detected" if detected else "Not Detected"} | '
                f'yaw={self.current_yaw:.3f} rad'
            )

    def destroy_node(self):
        if GPIO_AVAILABLE:
            try:
                GPIO.cleanup()
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = IRReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
