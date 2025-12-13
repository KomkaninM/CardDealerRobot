#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import spidev


class ButtonNode(Node):
    def __init__(self):
        super().__init__('button_node')

        # publish ไปให้ LCD
        self.pub = self.create_publisher(String, '/lcd1', 10)

        # MCP3008 SPI
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)          # bus 0, CE0
        self.spi.max_speed_hz = 1350000

        self.last_pressed = False
        self.create_timer(0.1, self.read_button)

        self.get_logger().info("Button node started (publish RIGHT -> /lcd1)")

    def read_adc(self, ch=0):
        adc = self.spi.xfer2([1, (8 + ch) << 4, 0])
        return ((adc[1] & 3) << 8) + adc[2]

    def read_button(self):
        value = self.read_adc(0)     # A0
        pressed = value < 50         # RIGHT ≈ 0V

        # publish เฉพาะ "ตอนกด"
        if pressed and not self.last_pressed:
            msg = String()
            msg.data = "RIGHT"
            self.pub.publish(msg)
            self.get_logger().info("RIGHT pressed → published to /lcd1")

        self.last_pressed = pressed

    def destroy_node(self):
        try:
            self.spi.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = ButtonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
