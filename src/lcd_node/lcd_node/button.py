#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool  # [CHANGED] Import Bool instead of String
import RPi.GPIO as GPIO

class ButtonNode(Node):
    def __init__(self):
        super().__init__('button_node')

        # 1. Setup Publisher to /lcd/button (Using Bool type)
        self.pub = self.create_publisher(Bool, '/lcd/button', 10)  # [CHANGED]

        # 2. Setup GPIO
        self.button_pin = 23
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # 3. State tracking
        self.last_pressed = False
        
        # 4. Create a timer to check the button every 0.1 seconds
        self.create_timer(0.1, self.read_button)

        self.get_logger().info(f"Button node started on GPIO {self.button_pin}")
        self.get_logger().info("Published to: /lcd/button")

    def read_button(self):
        # Read input: LOW (0) means pressed because of Pull-Up resistor
        input_state = GPIO.input(self.button_pin)
        pressed = (input_state == GPIO.LOW)

        # Publish only on fresh press (Rising Edge logic)
        if pressed and not self.last_pressed:
            msg = Bool()
            msg.data = True  # [CHANGED] Set data to boolean True
            self.pub.publish(msg)
            self.get_logger().info("Button Pressed")

        self.last_pressed = pressed

    def destroy_node(self):
        try:
            GPIO.cleanup()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
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