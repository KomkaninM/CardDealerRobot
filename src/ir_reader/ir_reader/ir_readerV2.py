#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String, Float32
from nav_msgs.msg import Odometry

try:
    import RPi.GPIO as GPIO
    GPIO_OK = True
except Exception:
    GPIO_OK = False


def quat_to_yaw_rad(x, y, z, w) -> float:
    # yaw from quaternion
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class IRReaderNode(Node):
    def __init__(self):
        super().__init__('ir_reader_node')

        # Parameters
        self.declare_parameter('ir_gpio_bcm', 17)
        self.declare_parameter('active_low', True)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('also_publish_lcd', True)
        self.declare_parameter('odom_topic', '/odom')

        self.ir_gpio = int(self.get_parameter('ir_gpio_bcm').value)
        self.active_low = bool(self.get_parameter('active_low').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.also_publish_lcd = bool(self.get_parameter('also_publish_lcd').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)

        self.last_state = None
        self.latest_yaw_deg = None  # updated by /odom

        # Publishers
        self.pub_det = self.create_publisher(Bool, '/ir_reader/player_detection', 10)
        self.pub_lcd = self.create_publisher(String, '/lcd/display', 10)
        self.pub_angle = self.create_publisher(Float32, '/ir_reader/player_angle', 10)

        # Subscriber: /odom
        self.sub_odom = self.create_subscription(
            Odometry,
            'odom',
            self.odom_cb,
            10)

        # GPIO init
        if not GPIO_OK:
            self.get_logger().warning("RPi.GPIO not available (run on Raspberry Pi or install RPi.GPIO)")
            # ยังให้ node ทำงานต่อได้เพื่อรับ /odom และ publish ได้ ถ้าอยาก
            # แต่การอ่าน IR จะไม่ทำงาน
        else:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.ir_gpio, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        period = 1.0 / max(self.rate_hz, 1.0)
        self.create_timer(period, self.tick)

        self.get_logger().info(
            f"IR Reader started | GPIO={self.ir_gpio} | active_low={self.active_low} | odom={self.odom_topic}"
        )

    def odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw_rad = quat_to_yaw_rad(q.x, q.y, q.z, q.w)
        yaw_deg = math.degrees(yaw_rad)

        # optional: normalize to [-180, 180)
        yaw_deg = (yaw_deg + 180.0) % 360.0 - 180.0

        self.latest_yaw_deg = yaw_deg

    def read_detected(self) -> bool:
        raw = GPIO.input(self.ir_gpio)  # 0/1
        return (raw == 0) if self.active_low else (raw == 1)

    def tick(self):
        # ถ้าไม่มี GPIO ก็ไม่อ่าน IR
        if not GPIO_OK:
            return

        detected = self.read_detected()

        # publish only on change
        if detected != self.last_state:
            prev = self.last_state
            self.last_state = detected

            # 1) detection topic
            det_msg = Bool()
            det_msg.data = detected
            self.pub_det.publish(det_msg)

            text = "Detected" if detected else "Not Detected"
            self.get_logger().info(f"IR {text}")

            # 2) lcd topic
            if self.also_publish_lcd:
                lcd = String()
                lcd.data = f"IR STATUS|{text}"
                self.pub_lcd.publish(lcd)

            # 3) publish angle only when we just detected (False -> True)
            if (prev is False or prev is None) and detected is True:
                if self.latest_yaw_deg is None:
                    self.get_logger().warning("Detected but no /odom received yet; cannot publish angle.")
                else:
                    ang = Float32()
                    ang.data = float(self.latest_yaw_deg)
                    self.pub_angle.publish(ang)
                    self.get_logger().info(f"Publish player_angle = {ang.data:.2f} deg")

    def destroy_node(self):
        if GPIO_OK:
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
