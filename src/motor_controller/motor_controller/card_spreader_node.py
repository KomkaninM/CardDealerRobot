import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from motor_lib import run_forward_reverse


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

        self.get_logger().info("Card Spreader Node Started (Raspberry Pi REAL mode)")

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
            def sequence_finished():
                self.get_logger().info("Motor sequence finished")
                self.publish_status("STOPPED")
                run_forward_reverse(on_finished_cb=sequence_finished)
            self.publish_status("RUNNING")
        elif cmd == "STOP":
            motor_lib.stop()
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
        motor_lib.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()