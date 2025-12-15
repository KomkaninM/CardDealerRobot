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


def normalize_deg(deg: float) -> float:
    # [-180, 180)
    return (deg + 180.0) % 360.0 - 180.0


class IRReaderNode(Node):
    def __init__(self):
        super().__init__('ir_reader_node')

        # Parameters
        self.declare_parameter('ir_gpio_bcm', 17)
        self.declare_parameter('active_low', True)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('odom_topic', '/odom')

        self.ir_gpio = int(self.get_parameter('ir_gpio_bcm').value)
        self.active_low = bool(self.get_parameter('active_low').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)

        self.last_state = None
        self.latest_yaw_rad = None  # updated by /odom

        # Accumulator for "Detected window" circular mean
        self.sum_sin = 0.0
        self.sum_cos = 0.0
        self.n_samples = 0

        # Publishers
        self.pub_det = self.create_publisher(Bool, '/ir_reader/player_detection', 10)
        self.pub_angle = self.create_publisher(Float32, '/ir_reader/player_angle', 10)

        # Subscriber: /odom (use parameter!)
        self.sub_odom = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_cb,
            10
        )

        # GPIO init
        if not GPIO_OK:
            self.get_logger().warning("RPi.GPIO not available (run on Raspberry Pi or install RPi.GPIO)")
        else:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.ir_gpio, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        period = 1.0 / max(self.rate_hz, 1.0)
        self.create_timer(period, self.tick)

        self.get_logger().info(
            f"IR Reader Initialize | GPIO={self.ir_gpio} | active_low={self.active_low} | odom={self.odom_topic}"
        )
        self.get_logger().info(
            f"Published to /ir_reader/player_detection, /ir_reader/player_angle"
        )

    def odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw_rad = quat_to_yaw_rad(q.x, q.y, q.z, q.w)
        self.latest_yaw_rad = yaw_rad

    def read_detected(self) -> bool:
        raw = GPIO.input(self.ir_gpio)  # 0/1
        return (raw == 0) if self.active_low else (raw == 1)

    def reset_window_accumulator(self):
        self.sum_sin = 0.0
        self.sum_cos = 0.0
        self.n_samples = 0

    def accumulate_yaw(self):
        # Add latest yaw sample (circular-safe)
        if self.latest_yaw_rad is None:
            return
        self.sum_sin += math.sin(self.latest_yaw_rad)
        self.sum_cos += math.cos(self.latest_yaw_rad)
        self.n_samples += 1

    def compute_mean_yaw_deg(self):
        if self.n_samples <= 0:
            return None
        mean_rad = math.atan2(self.sum_sin, self.sum_cos)
        mean_deg = normalize_deg(math.degrees(mean_rad))
        return mean_deg

    def tick(self):
        if not GPIO_OK:
            return
        
        detected = self.read_detected()
        prev = self.last_state

        # While detected, keep accumulating yaw samples (every tick)
        if detected:
            self.accumulate_yaw()

        # Check for state change
        if detected != self.last_state:
            self.last_state = detected
            
            # --- Rising edge (False -> True) ---
            # Start a new window, but DO NOT publish yet
            if (prev is False or prev is None) and detected is True:
                self.reset_window_accumulator()
                self.accumulate_yaw()
                self.get_logger().info("Object Detected: Collecting data")

            # --- Falling edge (True -> False) ---
            # Object just left. Now we publish.
            if prev is True and detected is False:
                
                # 1. Publish Detection Trigger (Only True)
                det_msg = Bool()
                det_msg.data = True
                self.pub_det.publish(det_msg)
                # self.get_logger().info("Object Removed: Publishing Detection=True")

                # 2. Calculate and Publish Angle
                mean_deg = self.compute_mean_yaw_deg()
                if mean_deg is None:
                    self.get_logger().warning("End of detection, but no yaw samples collected; cannot publish angle.")
                else:
                    ang = Float32()
                    ang.data = float(mean_deg)
                    self.pub_angle.publish(ang)
                    self.get_logger().info(
                        f"Publish CENTER player_angle (mean) = {ang.data:.2f} deg "
                        f"(samples={self.n_samples})"
                    )
                
                # Clear window after publishing
                self.reset_window_accumulator()

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