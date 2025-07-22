#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class WorkingMecanumMover(Node):
    def __init__(self):
        super().__init__('working_mecanum_mover')
        # Use the same topic that worked in diagnostic
        self.publisher_ = self.create_publisher(Twist, 'controller/cmd_vel', 10)
        
        # Give time to connect
        time.sleep(0.5)
        self.get_logger().info("🤖 Working Mecanum Mover Ready!")
        
    def move_robot(self, linear_x=0.0, linear_y=0.0, angular_z=0.0, duration=3.0):
        """Move robot using the same pattern that worked in diagnostic"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        
        movement_type = "forward" if linear_x > 0 else "backward" if linear_x < 0 else "sideways" if linear_y != 0 else "rotating" if angular_z != 0 else "stopped"
        self.get_logger().info(f"🔄 Moving {movement_type}: x={linear_x:.2f}, y={linear_y:.2f}, z={angular_z:.2f} for {duration:.1f}s")
        
        # Send commands continuously at 10Hz (same as diagnostic)
        rate = 10  # Hz
        total_iterations = int(duration * rate)
        
        for i in range(total_iterations):
            self.publisher_.publish(twist)
            if i % (rate) == 0:  # Log every second
                self.get_logger().info(f"   Moving... {i//rate + 1}/{int(duration)} seconds")
            time.sleep(1.0 / rate)
        
        # Stop the robot
        self.stop_robot()
        self.get_logger().info(f"✅ {movement_type.capitalize()} movement completed")
    
    def stop_robot(self):
        """Stop the robot using same pattern as diagnostic"""
        stop_twist = Twist()  # All zeros
        
        # Send stop commands multiple times to ensure robot stops
        for i in range(10):
            self.publisher_.publish(stop_twist)
            time.sleep(0.1)
    
    # Simple movement functions
    def move_forward(self, speed=0.1, duration=3.0):
        """Move forward"""
        self.move_robot(linear_x=speed, duration=duration)
    
    def move_backward(self, speed=0.1, duration=3.0):
        """Move backward"""
        self.move_robot(linear_x=-speed, duration=duration)
    
    def move_left(self, speed=0.1, duration=3.0):
        """Move sideways left - Mecanum special capability!"""
        self.move_robot(linear_y=speed, duration=duration)
    
    def move_right(self, speed=0.1, duration=3.0):
        """Move sideways right - Mecanum special capability!"""
        self.move_robot(linear_y=-speed, duration=duration)
    
    def rotate_left(self, speed=0.3, duration=3.0):
        """Rotate counter-clockwise"""
        self.move_robot(angular_z=speed, duration=duration)
    
    def rotate_right(self, speed=0.3, duration=3.0):
        """Rotate clockwise"""
        self.move_robot(angular_z=-speed, duration=duration)
    
    def move_in_circle(self, linear_speed=0.1, angular_speed=0.3, duration=3.0):
        """Move in circle (forward + rotation)"""
        self.move_robot(linear_x=linear_speed, angular_z=angular_speed, duration=duration)
    
    def move_diagonal(self, forward_speed=0.1, side_speed=0.1, duration=3.0):
        """Move diagonally - Mecanum special!"""
        self.move_robot(linear_x=forward_speed, linear_y=side_speed, duration=duration)


def main(args=None):
    rclpy.init(args=args)
    mover = WorkingMecanumMover()
    
    try:
        # Test sequence - demonstrating Mecanum capabilities
        
        mover.get_logger().info("🚀 Starting Mecanum Movement Demo!")
        
        # Basic movements
        mover.move_forward(0.1, 3)
        time.sleep(1)
        
        mover.move_backward(0.1, 2)
        time.sleep(1)
        
        # Mecanum special: Sideways movement!
        mover.get_logger().info("🔥 Mecanum Special: Sideways Movement!")
        mover.move_left(0.1, 2)
        time.sleep(1)
        
        mover.move_right(0.1, 2)
        time.sleep(1)
        
        # Diagonal movement
        mover.get_logger().info("🔥 Mecanum Special: Diagonal Movement!")
        mover.move_diagonal(0.1, 0.1, 3)  # Forward-left diagonal
        time.sleep(1)
        
        # Circular movement
        mover.get_logger().info("🔄 Circular Movement!")
        mover.move_in_circle(0.1, 0.3, 4)
        time.sleep(1)
        
        # Rotation in place
        mover.get_logger().info("🔄 Rotation in Place!")
        mover.rotate_left(-0.3, 2)
        time.sleep(1)
        
        # mover.rotate_right(0.3, 2)
        # time.sleep(1)
        
        mover.get_logger().info("🎉 All Mecanum movements completed successfully!")
        
    except KeyboardInterrupt:
        mover.get_logger().info("🛑 Movement interrupted by user")
    
    finally:
        # Always stop robot on exit
        mover.stop_robot()
        mover.get_logger().info("👋 Mecanum mover shutdown complete")
        rclpy.shutdown()


if __name__ == '__main__':
    main() 