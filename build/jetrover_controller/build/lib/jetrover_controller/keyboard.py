#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select
import argparse
import time
import threading
import math

class MultiRobotTeleop(Node):
    def __init__(self, robot_namespace=''):
        # Create unique node name with namespace
        node_name = 'multi_robot_teleop'
        if robot_namespace:
            node_name = f'multi_robot_teleop_{robot_namespace}'
            
        super().__init__(node_name)
        
        # Build topic name with namespace
        if robot_namespace:
            topic_name = f'/{robot_namespace}/controller/cmd_vel'
        else:
            topic_name = 'controller/cmd_vel'
            
        self.publisher_ = self.create_publisher(Twist, topic_name, 10)
        self.robot_namespace = robot_namespace
        
        self.speed = 0.1
        self.turn_speed = 0.5  # Start conservative, use ] to increase
        
        print("🤖 Multi-Robot Keyboard Teleop")
        if robot_namespace:
            print(f"🏷️  Controlling Robot: {robot_namespace}")
        print("="*40)
        print("Controls:")
        print("  w/s: Forward/Backward")
        print("  a/d: Left/Right (Mecanum)")
        print("  q/e: Rotate Left/Right")
        print("  z/c: Diagonal movements")
        print("  +/-: Increase/Decrease speed")
        print("  [/]: Decrease/Increase turn speed")
        print("  r: Full 360° turn right (test)")
        print("  SPACE: Stop")
        print("  x: Exit")
        print("="*40)
        turn_time = (2 * math.pi) / self.turn_speed
        print(f"Current speed: {self.speed:.2f} m/s")
        print(f"Current turn speed: {self.turn_speed:.2f} rad/s (360° in {turn_time:.1f}s)")
        print(f"Publishing to: {topic_name}")
        
    def get_key(self):
        """Get single keypress without Enter"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.original_settings)
        return key
        
    def send_cmd(self, linear_x=0, linear_y=0, angular_z=0):
        """Send movement command"""
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.linear.y = float(linear_y)
        twist.angular.z = float(angular_z)
        self.publisher_.publish(twist)
        
    def full_turn_right(self):
        """Execute a full 360° turn to the right"""
        # Calculate time for 360° turn: 2π / angular_velocity
        turn_time = (2 * math.pi) / self.turn_speed
        
        print(f"🔄 Starting 360° turn right at {self.turn_speed:.2f} rad/s")
        print(f"⏱️  Expected time: {turn_time:.1f} seconds")
        
        # Start timing
        start_time = time.time()
        
        # Run in separate thread to not block keyboard input
        def execute_turn():
            # Send rotation commands at 10Hz for the calculated duration
            rate = 10  # Hz
            total_iterations = int(turn_time * rate)
            
            for i in range(total_iterations):
                self.send_cmd(angular_z=-self.turn_speed)  # Negative for right turn
                time.sleep(1.0 / rate)
            
            # Stop the robot
            self.send_cmd()  # Send stop command
            
            actual_time = time.time() - start_time
            print(f"✅ Turn completed! Actual time: {actual_time:.1f}s (Expected: {turn_time:.1f}s)")
        
        # Execute in background thread
        turn_thread = threading.Thread(target=execute_turn, daemon=True)
        turn_thread.start()
        
    def run(self):
        """Main control loop"""
        self.original_settings = termios.tcgetattr(sys.stdin)
        
        try:
            while True:
                key = self.get_key()
                
                if key == 'w':
                    print(f"⬆️  Forward ({self.speed:.2f})")
                    self.send_cmd(linear_x=self.speed)
                    
                elif key == 's':
                    print(f"⬇️  Backward ({self.speed:.2f})")
                    self.send_cmd(linear_x=-self.speed)
                    
                elif key == 'a':
                    print(f"⬅️  Left ({self.speed:.2f})")
                    self.send_cmd(linear_y=self.speed)
                    
                elif key == 'd':
                    print(f"➡️  Right ({self.speed:.2f})")
                    self.send_cmd(linear_y=-self.speed)
                    
                elif key == 'q':
                    print(f"🔄 Rotate Left ({self.turn_speed:.2f})")
                    self.send_cmd(angular_z=self.turn_speed)
                    
                elif key == 'e':
                    print(f"🔃 Rotate Right ({self.turn_speed:.2f})")
                    self.send_cmd(angular_z=-self.turn_speed)
                    
                elif key == 'z':
                    print(f"↖️  Diagonal Forward-Left")
                    self.send_cmd(linear_x=self.speed, linear_y=self.speed)
                    
                elif key == 'c':
                    print(f"↗️  Diagonal Forward-Right")
                    self.send_cmd(linear_x=self.speed, linear_y=-self.speed)
                    
                elif key == '+' or key == '=':
                    self.speed = min(0.3, self.speed + 0.05)
                    print(f"⚡ Speed: {self.speed:.2f}")
                    
                elif key == '-':
                    self.speed = max(0.05, self.speed - 0.05)
                    print(f"🐌 Speed: {self.speed:.2f}")
                    
                elif key == '[':
                    self.turn_speed = max(0.1, self.turn_speed - 0.2)
                    turn_time = (2 * math.pi) / self.turn_speed
                    print(f"🔄⬇️  Turn Speed: {self.turn_speed:.2f} rad/s (360° in {turn_time:.1f}s)")
                    
                elif key == ']':
                    self.turn_speed = min(4.0, self.turn_speed + 0.2)
                    turn_time = (2 * math.pi) / self.turn_speed
                    print(f"🔄⬆️  Turn Speed: {self.turn_speed:.2f} rad/s (360° in {turn_time:.1f}s)")
                    
                elif key == 'r':
                    self.full_turn_right()
                    
                elif key == ' ':
                    print("🛑 STOP")
                    self.send_cmd()
                    
                elif key == 'x' or key == '\x03':  # x or Ctrl+C
                    print("👋 Exiting...")
                    break
                    
                else:
                    print("❓ Unknown key. Press 'x' to exit.")
                    
        except KeyboardInterrupt:
            print("\n👋 Exiting...")
        finally:
            # Stop robot and restore terminal
            self.send_cmd()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.original_settings)

def main(args=None):
    # Parse ROS arguments
    rclpy.init(args=args)
    
    # Parse remaining arguments for robot namespace
    import sys
    robot_namespace = ''
    
    # Simple argument parsing for --robot
    if '--robot' in sys.argv:
        try:
            robot_idx = sys.argv.index('--robot')
            if robot_idx + 1 < len(sys.argv):
                robot_namespace = sys.argv[robot_idx + 1]
        except (ValueError, IndexError):
            pass
    
    try:
        controller = MultiRobotTeleop(robot_namespace)
        controller.run()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()