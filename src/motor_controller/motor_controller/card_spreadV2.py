import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import RPi.GPIO as GPIO
import time
import threading

# -------------------------
# PIN CONFIG
# -------------------------
IN1 = 20
IN2 = 16
ENA = 12

PWM_FREQ = 1000
SPEED = 90
SPEEDBACK = 100
FORWARD_TIME = 0.35
REVERSE_TIME = 0.25
PAUSE_TIME = 0

# -------------------------
# GPIO SETUP
# -------------------------
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)

pwm = GPIO.PWM(ENA, PWM_FREQ)
pwm.start(0)

# -------------------------
# MOTOR ACTIONS
# -------------------------
def forward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    pwm.ChangeDutyCycle(SPEED)

def reverse():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    pwm.ChangeDutyCycle(SPEEDBACK)

def stop():
    pwm.ChangeDutyCycle(0)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)

# -------------------------
# SEQUENCE
# -------------------------
def run_forward_reverse():
        # Forward
        forward()
        time.sleep(FORWARD_TIME)

        # Pause
        stop()
        time.sleep(PAUSE_TIME)

        # Reverse
        reverse()
        time.sleep(REVERSE_TIME)

        # Stop
        stop()

class CardSpreaderNode(Node):

    def __init__(self):
        super().__init__('card_spreader_node')

        self.cmd_sub = self.create_subscription(
            String,
            '/card_spreader/cmd',
            self.cmd_callback,
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/card_spreader/status',
            10
        )

        self.get_logger().info("Card Spreader Node Started")
        self.get_logger().info("Subscribed to: /card_spreader/cmd")

        # เริ่มต้นเป็น STOPPED
        self.publish_status("STOPPED")

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(f"Status → {text}")

    def cmd_callback(self, msg: String):
        cmd = msg.data.strip().upper()
        self.get_logger().info(f"Received CMD: {cmd}")

        if cmd == "START":
            run_forward_reverse()
            self.publish_status("RUNNING")
        elif cmd == "STOP":
            stop()
            self.publish_status("STOPPED")
        else:
            self.get_logger().warn("Unknown command")


def main(args=None):
    rclpy.init(args=args)
    node = CardSpreaderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
