#!/usr/bin/env python3
# encoding: utf-8
# Script to continuously silence the buzzer

import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import BuzzerState
import time

def main():
    # Initialize ROS2
    rclpy.init()
    
    # Create a simple node
    node = rclpy.create_node('buzzer_silencer_continuous')
    
    # Create publisher to buzzer topic
    buzzer_pub = node.create_publisher(
        BuzzerState,
        '/ros_robot_controller/set_buzzer',
        10  # QoS depth
    )
    
    # Create message to silence buzzer
    silence_msg = BuzzerState()
    silence_msg.freq = 0       # Set frequency to 0
    silence_msg.on_time = 0.0  # No on time
    silence_msg.off_time = 0.0 # No off time
    silence_msg.repeat = 0   # Set to 0 repeats to ensure silence
    
    # Give publisher time to initialize
    time.sleep(0.5)
    
    node.get_logger().info('Continuously silencing buzzer... Press Ctrl+C to stop.')
    
    try:
        while rclpy.ok():
            buzzer_pub.publish(silence_msg)
            # Publish at a rate of 10Hz
            time.sleep(0.1)  
            # Process ROS events, but don't block
            rclpy.spin_once(node, timeout_sec=0) 
    except KeyboardInterrupt:
        node.get_logger().info('Buzzer silencer stopped by user.')
    finally:
        # Send one last silence command just in case
        buzzer_pub.publish(silence_msg)
        time.sleep(0.1)
        
        # Clean up
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 