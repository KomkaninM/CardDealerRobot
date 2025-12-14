#!/usr/bin/env python3
import rclpy
from std_msgs.msg import Float32
import time
import random

def main(args=None):
    rclpy.init(args=args)
    
    # 1. Setup
    node = rclpy.create_node('fixed_angle_publisher')
    publisher = node.create_publisher(Float32, '/rotator/angle', 10)
    
    # 2. Prepare Data
    msg = Float32()
    msg.data = round(random.uniform(0.0, 180.0)) 
    
    # 3. Publish
    # Note: We sleep briefly to ensure the connection is ready (discovery time)
    time.sleep(0.5) 
    publisher.publish(msg)
    node.get_logger().info(f'Published: {msg.data}')

    # 4. The Magic: spin_once
    # This processes the "callbacks" one time. It ensures the message 
    # is actually handed over to the network layer (DDS) before we kill the script.
    # timeout_sec=1 means "try to work for up to 1 second, then move on".
    rclpy.spin_once(node, timeout_sec=1.0)

    # 5. Cleanup and Exit automatically
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()