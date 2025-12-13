#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

try:
    import RPi.GPIO as GPIO
    GPIO_OK = True
except Exception:
    GPIO_OK = False


class IRReaderNode(Node):
    def __init__(self):
        super().__init__('ir_reader_node')

        # Parameters
        self.declare_parameter('ir_gpio_bcm', 17)
        self.declare_parameter('active_low', True)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('also_publish_lcd', True)

        self.ir_gpio = int(self.get_parameter('ir_gpio_bcm').value)
        self.active_low = bool(self.get_parameter('active_low').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.also_publish_lcd = bool(self.get_parameter('also_publish_lcd').value)

        self.last_state = None

        # Publishers
        self.pub_det = self.create_publisher(Bool, '/ir_reader/player_detection', 10)
        self.pub_lcd = self.create_publisher(String, '/lcd/display', 10)

        # GPIO init
        if not GPIO_OK:
            self.get_logger().warning("RPi.GPIO not available")
            return

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.ir_gpio, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        period = 1.0 / max(self.rate_hz, 1.0)
        self.create_timer(period, self.tick)

        self.get_logger().info(
            f"IR Reader started | GPIO={self.ir_gpio} | active_low={self.active_low}"
        )

    def read_detected(self) -> bool:
        raw = GPIO.input(self.ir_gpio)  # 0/1
        return (raw == 0) if self.active_low else (raw == 1)

    def tick(self):
        if not GPIO_OK:
            return

        detected = self.read_detected()

        # publish only on change
        if detected != self.last_state:
            self.last_state = detected

            msg = Bool()
            msg.data = detected
            self.pub_det.publish(msg)

            text = "Detected" if detected else "Not Detected"
            self.get_logger().info(f"IR {text}")

            if self.also_publish_lcd:
                lcd = String()
                lcd.data = f"IR STATUS|{text}"
                self.pub_lcd.publish(lcd)

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

