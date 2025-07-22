#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.publisher_ = self.create_publisher(Twist, 'controller/cmd_vel', 10)
        
        self.speed = 0.1
        self.turn_speed = 0.3
        
        print("🤖 Mecanum Robot Keyboard Controller")
        print("="*40)
        print("Controls:")
        print("  w/s: Forward/Backward")
        print("  a/d: Left/Right (Mecanum)")
        print("  q/e: Rotate Left/Right")
        print("  z/c: Diagonal movements")
        print("  +/-: Increase/Decrease speed")
        print("  SPACE: Stop")
        print("  x: Exit")
        print("="*40)
        print(f"Current speed: {self.speed:.2f} m/s")
        
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

def main():
    rclpy.init()
    controller = KeyboardController()
    
    try:
        controller.run()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 