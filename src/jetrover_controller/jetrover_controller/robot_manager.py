#!/usr/bin/env python3
import rclpy
import time
import math
import argparse
import sys
import signal
from enum import Enum
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from jetrover_controller.move_arm import ArmController
from jetrover_controller.move_robot import RobotMover

# Global variable to track robot manager for signal handling
robot_manager_instance = None

def signal_handler(signum, frame):
    """Handle shutdown signals to stop the robot"""
    global robot_manager_instance
    if robot_manager_instance:
        print("\n🛑 Signal received - stopping robot...")
        robot_manager_instance.force_stop_robot()
        time.sleep(0.5)
        # Send additional stop commands
        stop_msg = Twist()
        for _ in range(10):
            robot_manager_instance.wheel_node.publisher_.publish(stop_msg)
            time.sleep(0.01)
    sys.exit(0)

class SafetyMode(Enum):
    NORMAL = 0
    STOP = 1
    NAVIGATE = 2

class RobotManager(Node):
    def __init__(self):
        super().__init__('robot_manager')
        
        # Initialize arm and wheel controllers
        self.arm_node = ArmController()
        self.wheel_node = RobotMover()
        
        # LIDAR safety setup
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.lidar_subscription = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, qos)

        # Safety parameters
        self.emergency_stop_distance = 0.3  # Meters - immediate stop (increased for safety)
        self.navigation_distance = 0.6    # Meters - start navigating around (increased for better planning)


        self.safety_mode = SafetyMode.NORMAL
        
        # Pre-allocate twist messages for reuse
        self.stop_cmd = Twist()
        self.nav_cmd = Twist()
        
        # Navigation parameters
        self.sector_count = 8  # Divide scan into sectors
        self.sectors = [float('inf')] * self.sector_count
        self.is_navigating = False
        self.obstacle_memory = {}
        self.max_memory_size = 100
        self.learning_rate = 0.2
        
        # Robot task state
        self.intended_action = None
        self.action_start_time = None
        self.action_duration = None
        self.is_task_active = False
        
        # Navigation timer (only when needed)
        self.nav_timer = None
        
        self.get_logger().info("Integrated Robot Manager with Safety started")

    def start_navigation_timer(self):
        """Start the navigation timer only when needed"""
        if self.nav_timer is None:
            self.nav_timer = self.create_timer(0.1, self.navigation_control)
            self.get_logger().debug("Navigation timer started")

    def stop_navigation_timer(self):
        """Stop the navigation timer when not needed"""
        if self.nav_timer:
            self.destroy_timer(self.nav_timer)
            self.nav_timer = None
            self.get_logger().debug("Navigation timer stopped")

    def lidar_callback(self, msg):
        """Process LIDAR data for safety"""
        if not msg.ranges:
            return
            
        # Filter valid distances once
        min_range = float('inf')
        valid_ranges = []
        for r in msg.ranges:
            if 0.02 < r < 8.0 and not math.isnan(r) and r != float('inf'):
                min_range = min(min_range, r)
                valid_ranges.append(r)
        
        if min_range == float('inf') or not valid_ranges:
            self.get_logger().warn("No valid LIDAR data received")
            return
            
        # Log current closest obstacle
        self.get_logger().debug(f"Closest obstacle: {min_range:.3f}m (emergency: {self.emergency_stop_distance:.3f}m, nav: {self.navigation_distance:.3f}m)")
            
        # Only process sectors if we'll need them for navigation
        if min_range < self.navigation_distance:
            self.process_sectors(msg)
        
        # Check for clear path ahead (front sectors)
        front_sectors = [0, 1, 7]  # Front, front-left, front-right
        front_clear = all(self.sectors[i] > self.navigation_distance for i in front_sectors if i < len(self.sectors))
        
        # Safety decision making
        if min_range < self.emergency_stop_distance:
            if self.safety_mode != SafetyMode.STOP:
                self.get_logger().warn(f"EMERGENCY STOP TRIGGERED: Obstacle at {min_range:.3f}m (threshold: {self.emergency_stop_distance:.3f}m)")
                self.safety_mode = SafetyMode.STOP
                self.emergency_stop()
            else:
                self.get_logger().debug(f"Already in emergency stop mode - obstacle at {min_range:.3f}m")
        elif min_range < self.navigation_distance:
            if self.safety_mode != SafetyMode.NAVIGATE:
                self.get_logger().info(f"NAVIGATION MODE: Obstacle at {min_range:.3f}m (threshold: {self.navigation_distance:.3f}m)")
                self.safety_mode = SafetyMode.NAVIGATE
                self.start_navigation()
            else:
                self.get_logger().debug(f"Continuing navigation - obstacle at {min_range:.3f}m")
        else:
            # Check if front path is clear and we have enough clearance
            if front_clear and min_range > self.navigation_distance * 1.2:
                if self.safety_mode != SafetyMode.NORMAL:
                    self.get_logger().info(f"PATH CLEAR - resuming normal operation (min distance: {min_range:.3f}m)")
                    self.safety_mode = SafetyMode.NORMAL
                    self.is_navigating = False
                    self.stop_navigation_timer()
                    # Resume intended action if we were doing something
                    if self.is_task_active and self.intended_action:
                        self.resume_intended_action()
            else:
                self.get_logger().debug(f"Path not fully clear yet - min: {min_range:.3f}m, front_clear: {front_clear}")

    def process_sectors(self, msg):
        """Process LIDAR data into sectors - only called when needed"""
        num_readings = len(msg.ranges)
        sector_size = num_readings // self.sector_count
        
        # Reset sectors
        for i in range(self.sector_count):
            self.sectors[i] = float('inf')
        
        # Divide scan into sectors more efficiently
        for i in range(self.sector_count):
            start_idx = i * sector_size
            end_idx = (i + 1) * sector_size if i < self.sector_count - 1 else num_readings
            
            # Find minimum in this sector
            sector_min = float('inf')
            for j in range(start_idx, end_idx):
                r = msg.ranges[j]
                if 0.02 < r < 8.0 and not math.isnan(r) and r != float('inf'):
                    sector_min = min(sector_min, r)
            
            self.sectors[i] = sector_min
            
            # Update obstacle memory with exponential smoothing
            if i not in self.obstacle_memory:
                if len(self.obstacle_memory) >= self.max_memory_size:
                    if self.obstacle_memory:
                        self.obstacle_memory.pop(next(iter(self.obstacle_memory)))
                self.obstacle_memory[i] = float('inf')
            
            if sector_min < self.navigation_distance:
                self.obstacle_memory[i] = (1 - self.learning_rate) * self.obstacle_memory[i] + self.learning_rate * sector_min
            else:
                self.obstacle_memory[i] = min(self.obstacle_memory[i] + 0.05, float('inf'))

        # Log obstacle detection with sector names
        self.log_obstacle_sectors()

    def get_sector_name(self, sector_id):
        """Convert sector ID to human-readable direction"""
        sector_names = {
            0: "FRONT",
            1: "FRONT-LEFT", 
            2: "LEFT",
            3: "BACK-LEFT",
            4: "BACK", 
            5: "BACK-RIGHT",
            6: "RIGHT",
            7: "FRONT-RIGHT"
        }
        return sector_names.get(sector_id, f"SECTOR-{sector_id}")

    def log_obstacle_sectors(self):
        """Log which sectors have obstacles detected"""
        obstacle_sectors = []
        for i, distance in enumerate(self.sectors):
            if distance < self.navigation_distance:
                obstacle_sectors.append(f"{self.get_sector_name(i)}({distance:.2f}m)")
        
        if obstacle_sectors:
            self.get_logger().info(f"Obstacles detected in: {', '.join(obstacle_sectors)}")

    def emergency_stop(self):
        """Immediately stop the robot"""
        self.get_logger().error("🛑 EMERGENCY STOP ACTIVATED! 🛑")
        
        # Send multiple stop commands to ensure it stops
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        
        # Publish stop command multiple times for reliability
        for i in range(10):
            self.wheel_node.publisher_.publish(stop_msg)
            if i < 5:  # Add small delays for the first few commands
                time.sleep(0.005)
        
        # Also call the wheel node's stop method
        self.wheel_node.stop()
        
        self.get_logger().error("Emergency stop commands sent!")
        self.stop_navigation_timer()
        
        # Cancel any active tasks
        self.is_task_active = False
        self.intended_action = None
        
        # Keep sending stop commands for a bit longer
        def keep_stopping():
            for _ in range(5):
                self.wheel_node.publisher_.publish(stop_msg)
                time.sleep(0.01)
        
        # Schedule additional stop commands
        self.create_timer(0.1, keep_stopping)

    def start_navigation(self):
        """Begin the navigation around obstacles"""
        if not self.is_navigating:
            self.is_navigating = True
            self.start_navigation_timer()
            self.get_logger().info("Starting obstacle navigation")

    def navigation_control(self):
        """Timer callback for navigation around obstacles"""
        if not self.is_navigating or self.safety_mode == SafetyMode.STOP:
            return
        
        # Find the best direction to move
        best_direction = self.find_best_navigation_direction()
        
        if best_direction is not None:
            angular_speed, linear_speed, reason = best_direction
            
            # Apply navigation command
            self.nav_cmd.linear.x = linear_speed
            self.nav_cmd.angular.z = angular_speed
            self.wheel_node.publisher_.publish(self.nav_cmd)
            
            direction_name = "LEFT" if angular_speed > 0.1 else "RIGHT" if angular_speed < -0.1 else "FORWARD"
            self.get_logger().info(f"Navigating {direction_name}: {reason} (linear={linear_speed:.2f}, angular={angular_speed:.2f})")
        else:
            # No safe direction found - stop and reassess
            self.nav_cmd.linear.x = 0.0
            self.nav_cmd.angular.z = 0.0
            self.wheel_node.publisher_.publish(self.nav_cmd)
            self.get_logger().warn("No safe navigation direction found - stopping")

    def find_best_navigation_direction(self):
        """Find the best direction to navigate around obstacles"""
        # Check each sector for clearance
        sector_clearances = []
        for i in range(self.sector_count):
            if i in self.obstacle_memory:
                clearance = self.obstacle_memory[i]
            else:
                clearance = float('inf')
            sector_clearances.append(clearance)
        
        # Define sector preferences (prefer front directions)
        sector_preferences = {
            0: 1.0,    # FRONT - highest preference
            1: 0.8,    # FRONT-LEFT
            7: 0.8,    # FRONT-RIGHT
            2: 0.6,    # LEFT
            6: 0.6,    # RIGHT
            3: 0.4,    # BACK-LEFT
            5: 0.4,    # BACK-RIGHT
            4: 0.2     # BACK - lowest preference
        }
        
        # Find sectors with sufficient clearance
        safe_sectors = []
        for i in range(self.sector_count):
            if sector_clearances[i] > self.navigation_distance * 1.1:  # Require extra clearance
                safe_sectors.append((i, sector_clearances[i], sector_preferences.get(i, 0.1)))
        
        if not safe_sectors:
            # No safe sectors - look for the safest direction to turn
            return self.find_escape_direction(sector_clearances)
        
        # Sort by preference and clearance
        safe_sectors.sort(key=lambda x: (-x[2], -x[1]))  # Sort by preference desc, then clearance desc
        best_sector = safe_sectors[0][0]
        
        # Convert sector to movement commands
        return self.sector_to_movement(best_sector, f"heading towards clear sector {self.get_sector_name(best_sector)}")

    def find_escape_direction(self, sector_clearances):
        """Find the best direction to escape when all paths are blocked"""
        # Find the sector with maximum clearance
        max_clearance = max(sector_clearances)
        if max_clearance < 0.05:  # Too close everywhere
            return None
        
        best_sectors = [i for i, clearance in enumerate(sector_clearances) if clearance == max_clearance]
        
        # Prefer side directions over back directions for escape
        preference_order = [1, 7, 2, 6, 0, 3, 5, 4]  # front-left, front-right, left, right, front, back-left, back-right, back
        
        for preferred in preference_order:
            if preferred in best_sectors:
                return self.sector_to_movement(preferred, f"escaping towards sector {self.get_sector_name(preferred)} (clearance: {max_clearance:.2f}m)")
        
        # Fallback to first available sector
        return self.sector_to_movement(best_sectors[0], f"escaping towards sector {self.get_sector_name(best_sectors[0])}")

    def sector_to_movement(self, sector_id, reason):
        """Convert a sector ID to movement commands"""
        # Calculate sector angle (0 = front, clockwise)
        sector_angle = sector_id * 2 * math.pi / self.sector_count
        
        # Convert to angular velocity
        # Positive angular = left turn, negative = right turn
        target_angular = -sector_angle  # Negative because we want to turn towards the sector
        
        # Normalize angle to [-pi, pi]
        while target_angular > math.pi:
            target_angular -= 2 * math.pi
        while target_angular < -math.pi:
            target_angular += 2 * math.pi
        
        # Scale angular velocity
        angular_speed = 0.6 * target_angular
        angular_speed = max(-0.8, min(0.8, angular_speed))  # Limit turning speed
        
        # Set linear speed based on how much we need to turn
        if abs(angular_speed) > 0.4:
            linear_speed = 0.05  # Slow when turning sharply
        elif abs(angular_speed) > 0.2:
            linear_speed = 0.08  # Medium speed for moderate turns
        else:
            linear_speed = 0.12  # Faster when going mostly straight
        
        return angular_speed, linear_speed, reason

    def safe_move_in_circle(self, linear_speed, angular_speed, duration):
        """Move in circle with safety monitoring"""
        if self.safety_mode != SafetyMode.NORMAL:
            self.get_logger().warn("Cannot start movement - safety system engaged")
            return
            
        # Store the intended action for resuming
        self.intended_action = {
            'type': 'circle',
            'linear_speed': linear_speed,
            'angular_speed': angular_speed,
            'duration': duration
        }
        self.action_start_time = time.time()
        self.action_duration = duration
        self.is_task_active = True
        
        self.get_logger().info(f"Starting safe movement: linear={linear_speed}, angular={angular_speed}, duration={duration}s")
        self.wheel_node.move_in_circle(linear_speed, angular_speed, duration)
        
        # Mark task as completed after duration - create a one-shot timer
        def complete_task():
            self.mark_task_complete()
        
        self.completion_timer = self.create_timer(duration, complete_task)
        # Make it a one-shot timer
        self.completion_timer.timer_period_ns = 0

    def mark_task_complete(self):
        """Mark the current task as completed"""
        self.is_task_active = False
        self.intended_action = None
        
        # Cancel completion timer if it exists
        if hasattr(self, 'completion_timer') and self.completion_timer:
            self.destroy_timer(self.completion_timer)
            self.completion_timer = None
            
        self.get_logger().info("Task completed")

    def force_stop_robot(self):
        """Force stop the robot immediately - used for cleanup"""
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        
        # Send multiple stop commands
        for _ in range(10):
            self.wheel_node.publisher_.publish(stop_msg)
            time.sleep(0.01)
            
        self.get_logger().info("Robot force stopped")
        
        # Cancel all tasks and timers
        self.is_task_active = False
        self.intended_action = None
        self.is_navigating = False
        self.stop_navigation_timer()
        
        if hasattr(self, 'completion_timer') and self.completion_timer:
            self.destroy_timer(self.completion_timer)
            self.completion_timer = None

    def resume_intended_action(self):
        """Resume the intended action after obstacle is cleared"""
        if not self.intended_action or not self.is_task_active:
            return
            
        # Calculate remaining time
        elapsed_time = time.time() - self.action_start_time
        remaining_time = self.action_duration - elapsed_time
        
        if remaining_time > 0:
            self.get_logger().info(f"Resuming movement for {remaining_time:.1f}s")
            action = self.intended_action
            self.wheel_node.move_in_circle(
                action['linear_speed'], 
                action['angular_speed'], 
                remaining_time
            )
            # Update the timer for completion
            def complete_task():
                self.mark_task_complete()
            
            # Cancel existing timer if any
            if hasattr(self, 'completion_timer') and self.completion_timer:
                self.destroy_timer(self.completion_timer)
                
            self.completion_timer = self.create_timer(remaining_time, complete_task)
            self.completion_timer.timer_period_ns = 0  # One-shot timer
        else:
            self.get_logger().info("Task time expired during obstacle avoidance")
            self.mark_task_complete()

def main(args=None):
    global robot_manager_instance
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Robot Manager with integrated LIDAR safety")
    parser.add_argument('--linear', '-l', type=float, default=0.1,
                        help='Linear velocity in m/s (default: 0.1)')
    parser.add_argument('--angular', '-a', type=float, default=0.0,
                        help='Angular velocity in rad/s (default: 0.0)')
    parser.add_argument('--duration', '-d', type=float, default=5.0,
                        help='Movement duration in seconds (default: 5.0)')
    parser.add_argument('--emergency-stop', '-e', type=float, default=0.3,
                        help='Emergency stop distance in meters (default: 0.3)')
    parser.add_argument('--navigation', '-n', type=float, default=0.5,
                        help='Navigation start distance in meters (default: 0.5)')
    
    # Filter out ROS arguments and parse only our arguments
    if args is None:
        # Filter out ROS args
        filtered_args = []
        skip_next = False
        for i, arg in enumerate(sys.argv[1:]):
            if skip_next:
                skip_next = False
                continue
            if arg.startswith('--ros-args') or arg.startswith('__'):
                # Skip ROS arguments
                if '=' not in arg and i + 1 < len(sys.argv[1:]):
                    skip_next = True  # Skip the next argument as well
                continue
            filtered_args.append(arg)
        parsed_args = parser.parse_args(filtered_args)
    else:
        parsed_args = parser.parse_args(args)
    
    # Initialize ROS
    rclpy.init(args=sys.argv)
    
    # Create the integrated robot manager
    robot_manager = RobotManager()
    robot_manager_instance = robot_manager  # Set global instance for signal handling
    
    # Apply custom safety distances if provided
    if parsed_args.emergency_stop != 0.3:
        robot_manager.emergency_stop_distance = parsed_args.emergency_stop
        robot_manager.get_logger().info(f"Custom emergency stop distance: {parsed_args.emergency_stop}m")
    
    if parsed_args.navigation != 0.5:
        robot_manager.navigation_distance = parsed_args.navigation
        robot_manager.get_logger().info(f"Custom navigation distance: {parsed_args.navigation}m")
    
    # Give some time for LIDAR to initialize
    time.sleep(1.0)
    
    # Log the movement parameters
    robot_manager.get_logger().info(f"Movement parameters: linear={parsed_args.linear} m/s, "
                                  f"angular={parsed_args.angular} rad/s, duration={parsed_args.duration}s")
    
    # Execute the planned movement with safety using custom parameters
    robot_manager.safe_move_in_circle(parsed_args.linear, parsed_args.angular, parsed_args.duration)
    
    # Uncomment the following for more complex behaviors:
    # default_pose = {
    #     1: 500,  # joint1
    #     2: 500,  # joint2
    #     3: 500,
    #     4: 500,
    #     5: 500,
    #     10: 500  # gripper
    # }
    
    # rclpy.spin_once(robot_manager.arm_node, timeout_sec=1.0)
    
    # if not robot_manager.arm_node.is_default_pose(default_pose):
    #     print("🔁 Robot is NOT in default pose. Resetting...")
    #     robot_manager.arm_node.default_move()
    #     time.sleep(2)
    # else:
    #     print("✅ Already in default pose!")
    
    try:
        # Keep the node running to handle LIDAR callbacks and safety
        rclpy.spin(robot_manager)
    except KeyboardInterrupt:
        print("🛑 Interrupted!")
    finally:
        print("🛑 Stopping...")
        # Force stop the robot multiple times to ensure it stops
        robot_manager.force_stop_robot()
        time.sleep(0.5)  # Give time for stop commands to be processed
        
        # Additional stop commands for safety
        stop_msg = Twist()
        for _ in range(20):
            robot_manager.wheel_node.publisher_.publish(stop_msg)
            time.sleep(0.01)
            
        robot_manager.wheel_node.stop()
        robot_manager.destroy_node()
        rclpy.shutdown()
        print("✅ Robot stopped and node destroyed")

if __name__ == '__main__':
    # Set up signal handler
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    main()


