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
import json
import os
from datetime import datetime

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
        
        # Position tracking for choreography
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.last_movement_time = time.time()
        
        # Recording functionality
        self.recording_mode = False
        self.current_move_sequence = []
        self.recorded_moves = {}
        self.moves_file = f"recorded_moves_{robot_namespace}.json" if robot_namespace else "recorded_moves.json"
        self.load_recorded_moves()
        
        # Position tracking for choreography
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.last_movement_time = time.time()
        
        # Choreography functionality
        self.choreography_mode = False
        self.choreography_moves = {}
        self.choreography_file = f"choreography_{robot_namespace}.json" if robot_namespace else "choreography.json"
        self.load_choreography()
        
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
        print("")
        print("🎬 Recording Controls:")
        print("  R: Start/Stop recording mode")
        print("  p: Play recorded move")
        print("  l: List recorded moves")
        print("  d: Delete recorded move")
        print("")
        print("🎭 Choreography Controls:")
        print("  C: Start/Stop choreography mode")
        print("  t: Create timed movement")
        print("  s: Create spread & align movement")
        print("  a: Create advanced multi-phase movement")
        print("  P: Play choreography")
        print("  L: List choreographies")
        print("  D: Delete choreography")
        print("  i: Show current position")
        print("="*40)
        turn_time = (2 * math.pi) / self.turn_speed
        print(f"Current speed: {self.speed:.2f} m/s")
        print(f"Current turn speed: {self.turn_speed:.2f} rad/s (360° in {turn_time:.1f}s)")
        print(f"Publishing to: {topic_name}")
        
    def update_position(self, linear_x, linear_y, angular_z, duration):
        """Update robot position based on movement"""
        # Simple position tracking (assumes constant velocity during duration)
        # In a real implementation, you'd use odometry or SLAM data
        
        # Update position based on linear movement
        self.current_x += linear_x * duration
        self.current_y += linear_y * duration
        
        # Update orientation based on angular movement
        self.current_theta += angular_z * duration
        
        # Keep theta in [-π, π]
        self.current_theta = math.atan2(math.sin(self.current_theta), math.cos(self.current_theta))
        
        self.last_movement_time = time.time()
        
    def get_position(self):
        """Get current robot position"""
        return {
            'x': self.current_x,
            'y': self.current_y,
            'theta': self.current_theta,
            'timestamp': self.last_movement_time
        }
        
    def reset_position(self):
        """Reset position to origin"""
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.last_movement_time = time.time()
        print("🔄 Position reset to origin (0, 0, 0)")
        
    def load_recorded_moves(self):
        """Load recorded moves from JSON file"""
        try:
            if os.path.exists(self.moves_file):
                with open(self.moves_file, 'r') as f:
                    self.recorded_moves = json.load(f)
                print(f"📂 Loaded {len(self.recorded_moves)} recorded moves from {self.moves_file}")
        except Exception as e:
            print(f"⚠️  Could not load recorded moves: {e}")
            self.recorded_moves = {}
            
    def save_recorded_moves(self):
        """Save recorded moves to JSON file"""
        try:
            with open(self.moves_file, 'w') as f:
                json.dump(self.recorded_moves, f, indent=2)
            print(f"💾 Saved {len(self.recorded_moves)} moves to {self.moves_file}")
        except Exception as e:
            print(f"❌ Error saving moves: {e}")
            
    def load_choreography(self):
        """Load choreography from JSON file"""
        try:
            if os.path.exists(self.choreography_file):
                with open(self.choreography_file, 'r') as f:
                    self.choreography_moves = json.load(f)
                print(f"📂 Loaded {len(self.choreography_moves)} choreographies from {self.choreography_file}")
        except Exception as e:
            print(f"⚠️  Could not load choreography: {e}")
            self.choreography_moves = {}
            
    def save_choreography(self):
        """Save choreography to JSON file"""
        try:
            with open(self.choreography_file, 'w') as f:
                json.dump(self.choreography_moves, f, indent=2)
            print(f"💾 Saved {len(self.choreography_moves)} choreographies to {self.choreography_file}")
        except Exception as e:
            print(f"❌ Error saving choreography: {e}")
            
    def update_position(self, linear_x, linear_y, angular_z, duration):
        """Update robot position based on movement"""
        # Simple position tracking (assumes constant velocity during duration)
        # In a real implementation, you'd use odometry or SLAM data
        
        # Update position based on linear movement
        self.current_x += linear_x * duration
        self.current_y += linear_y * duration
        
        # Update orientation based on angular movement
        self.current_theta += angular_z * duration
        
        # Keep theta in [-π, π]
        self.current_theta = math.atan2(math.sin(self.current_theta), math.cos(self.current_theta))
        
        self.last_movement_time = time.time()
        
    def get_position(self):
        """Get current robot position"""
        return {
            'x': self.current_x,
            'y': self.current_y,
            'theta': self.current_theta,
            'timestamp': self.last_movement_time
        }
        
    def reset_position(self):
        """Reset position to origin"""
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.last_movement_time = time.time()
        print("🔄 Position reset to origin (0, 0, 0)")
        
    def show_position(self):
        """Show current robot position"""
        pos = self.get_position()
        print(f"📍 Current Position:")
        print(f"   X: {pos['x']:.3f} m")
        print(f"   Y: {pos['y']:.3f} m")
        print(f"   θ: {math.degrees(pos['theta']):.1f}°")
        print(f"   Last update: {time.time() - pos['timestamp']:.1f}s ago")
            
    def create_advanced_multi_phase_movement(self):
        """Create an advanced multi-phase movement choreography"""
        print("🎯 Creating Advanced Multi-Phase Movement")
        print("This creates a complex movement with primary and secondary phases")
        print("Robots move in opposite directions with position tracking and alignment")
        
        try:
            primary_time = float(input("Primary movement time (seconds): "))
            secondary_time = float(input("Secondary movement time (seconds): "))
            speed = float(input("Speed (m/s): "))
            
            choreography_name = input("Choreography name: ").strip()
            if not choreography_name:
                print("❌ No name provided")
                return
                
            # Create choreography data
            choreography_data = {
                'type': 'advanced_multi_phase',
                'primary_time': primary_time,
                'secondary_time': secondary_time,
                'speed': speed,
                'timestamp': datetime.now().isoformat(),
                'robot_namespace': self.robot_namespace
            }
            
            self.choreography_moves[choreography_name] = choreography_data
            self.save_choreography()
            print(f"✅ Created advanced multi-phase movement: {choreography_name}")
            print(f"   Primary: {primary_time}s, Secondary: {secondary_time}s, Speed: {speed} m/s")
            print(f"   Robot 1: Right→Left, Robot 2: Left→Right")
            
        except ValueError:
            print("❌ Invalid input")
            
    def create_timed_movement(self):
        """Create a timed movement choreography"""
        print("⏱️  Creating Timed Movement")
        print("Enter movement parameters:")
        
        try:
            duration = float(input("Duration (seconds): "))
            speed = float(input("Speed (m/s): "))
            direction = input("Direction (left/right/forward/backward): ").lower()
            
            if direction not in ['left', 'right', 'forward', 'backward']:
                print("❌ Invalid direction")
                return
                
            choreography_name = input("Choreography name: ").strip()
            if not choreography_name:
                print("❌ No name provided")
                return
                
            # Create choreography data
            choreography_data = {
                'type': 'timed_movement',
                'duration': duration,
                'speed': speed,
                'direction': direction,
                'timestamp': datetime.now().isoformat(),
                'robot_namespace': self.robot_namespace
            }
            
            self.choreography_moves[choreography_name] = choreography_data
            self.save_choreography()
            print(f"✅ Created timed movement: {choreography_name}")
            
        except ValueError:
            print("❌ Invalid input")
            
    def create_spread_align_movement(self):
        """Create a spread and align movement choreography"""
        print("🔄 Creating Spread & Align Movement")
        print("This creates synchronized movement where robots move in opposite directions")
        print("and then align back to the same vertical line")
        
        try:
            duration = float(input("Movement duration (seconds): "))
            speed = float(input("Speed (m/s): "))
            
            choreography_name = input("Choreography name: ").strip()
            if not choreography_name:
                print("❌ No name provided")
                return
                
            # Create choreography data
            choreography_data = {
                'type': 'spread_align',
                'duration': duration,
                'speed': speed,
                'timestamp': datetime.now().isoformat(),
                'robot_namespace': self.robot_namespace
            }
            
            self.choreography_moves[choreography_name] = choreography_data
            self.save_choreography()
            print(f"✅ Created spread & align movement: {choreography_name}")
            print(f"   Duration: {duration}s, Speed: {speed} m/s")
            print(f"   Robot 1 will move left, Robot 2 will move right")
            
        except ValueError:
            print("❌ Invalid input")
            
    def play_choreography(self, choreography_name):
        """Play a choreography movement"""
        if choreography_name not in self.choreography_moves:
            print(f"❌ Choreography '{choreography_name}' not found")
            return
            
        choreography = self.choreography_moves[choreography_name]
        choreography_type = choreography['type']
        
        print(f"🎭 Playing choreography: {choreography_name} ({choreography_type})")
        
        if choreography_type == 'timed_movement':
            self.play_timed_movement(choreography)
        elif choreography_type == 'spread_align':
            self.play_spread_align_movement(choreography)
        elif choreography_type == 'advanced_multi_phase':
            self.play_advanced_multi_phase_movement(choreography)
        else:
            print(f"❌ Unknown choreography type: {choreography_type}")
            
    def play_advanced_multi_phase_movement(self, choreography):
        """Play an advanced multi-phase movement"""
        primary_time = choreography['primary_time']
        secondary_time = choreography['secondary_time']
        speed = choreography['speed']
        
        print(f"🎯 Executing advanced multi-phase movement")
        print(f"   Primary: {primary_time}s, Secondary: {secondary_time}s, Speed: {speed} m/s")
        
        # Determine robot role based on namespace
        is_robot1 = 'robot1' in self.robot_namespace.lower() or self.robot_namespace == ''
        
        if is_robot1:
            print(f"🤖 {self.robot_namespace or 'Robot 1'} - Phase 1: RIGHT, Phase 2: LEFT")
            
            # Phase 1: Move RIGHT for primary_time
            print(f"   Phase 1: Moving RIGHT for {primary_time}s")
            start_time = time.time()
            while time.time() - start_time < primary_time:
                self.send_cmd(linear_y=-speed)  # Right movement
                time.sleep(0.1)
            self.update_position(0, -speed, 0, primary_time)
            
            # Phase 2: Move LEFT for secondary_time
            print(f"   Phase 2: Moving LEFT for {secondary_time}s")
            start_time = time.time()
            while time.time() - start_time < secondary_time:
                self.send_cmd(linear_y=speed)  # Left movement
                time.sleep(0.1)
            self.update_position(0, speed, 0, secondary_time)
            
        else:
            print(f"🤖 {self.robot_namespace} - Phase 1: LEFT, Phase 2: RIGHT")
            
            # Phase 1: Move LEFT for primary_time
            print(f"   Phase 1: Moving LEFT for {primary_time}s")
            start_time = time.time()
            while time.time() - start_time < primary_time:
                self.send_cmd(linear_y=speed)  # Left movement
                time.sleep(0.1)
            self.update_position(0, speed, 0, primary_time)
            
            # Phase 2: Move RIGHT for secondary_time
            print(f"   Phase 2: Moving RIGHT for {secondary_time}s")
            start_time = time.time()
            while time.time() - start_time < secondary_time:
                self.send_cmd(linear_y=-speed)  # Right movement
                time.sleep(0.1)
            self.update_position(0, -speed, 0, secondary_time)
        
        # Stop and show final position
        self.send_cmd()
        final_pos = self.get_position()
        print(f"✅ Completed advanced multi-phase movement")
        print(f"   Final position: x={final_pos['x']:.3f}, y={final_pos['y']:.3f}, θ={math.degrees(final_pos['theta']):.1f}°")
        
        # Calculate alignment movement if needed
        if abs(final_pos['y']) > 0.01:  # If not aligned (within 1cm)
            alignment_time = abs(final_pos['y']) / speed
            alignment_direction = -1 if final_pos['y'] > 0 else 1
            print(f"🔄 Aligning to vertical line (moving {'left' if alignment_direction > 0 else 'right'} for {alignment_time:.2f}s)")
            
            start_time = time.time()
            while time.time() - start_time < alignment_time:
                self.send_cmd(linear_y=alignment_direction * speed)
                time.sleep(0.1)
            self.send_cmd()
            self.update_position(0, alignment_direction * speed, 0, alignment_time)
            
            final_pos = self.get_position()
            print(f"✅ Alignment complete. Final position: x={final_pos['x']:.3f}, y={final_pos['y']:.3f}")
            
    def play_timed_movement(self, choreography):
        """Play a timed movement"""
        duration = choreography['duration']
        speed = choreography['speed']
        direction = choreography['direction']
        
        print(f"⏱️  Executing {direction} movement for {duration}s at {speed} m/s")
        
        # Determine movement command based on direction
        linear_x, linear_y = 0, 0
        if direction == 'forward':
            linear_x = speed
        elif direction == 'backward':
            linear_x = -speed
        elif direction == 'left':
            linear_y = speed
        elif direction == 'right':
            linear_y = -speed
            
        # Execute movement
        start_time = time.time()
        while time.time() - start_time < duration:
            self.send_cmd(linear_x=linear_x, linear_y=linear_y)
            time.sleep(0.1)  # 10Hz control loop
            
        # Update position and stop
        self.update_position(linear_x, linear_y, 0, duration)
        self.send_cmd()
        print(f"✅ Completed timed movement")
        
    def play_spread_align_movement(self, choreography):
        """Play a spread and align movement"""
        duration = choreography['duration']
        speed = choreography['speed']
        
        print(f"🔄 Executing spread & align movement for {duration}s at {speed} m/s")
        
        # Determine movement direction based on robot namespace
        # For this example, we'll assume robot1 moves left, robot2 moves right
        # You can modify this logic based on your robot naming convention
        if 'robot1' in self.robot_namespace.lower() or self.robot_namespace == '':
            # Robot 1 moves left
            linear_y = speed
            print(f"🤖 {self.robot_namespace or 'Robot 1'} moving LEFT")
        else:
            # Robot 2 moves right
            linear_y = -speed
            print(f"🤖 {self.robot_namespace} moving RIGHT")
            
        # Execute movement
        start_time = time.time()
        while time.time() - start_time < duration:
            self.send_cmd(linear_y=linear_y)
            time.sleep(0.1)  # 10Hz control loop
            
        # Update position, stop and align
        self.update_position(0, linear_y, 0, duration)
        self.send_cmd()
        print(f"✅ Completed spread & align movement")
        print(f"   Robots should now be aligned at the same vertical line")
        
    def list_choreography(self):
        """List all choreography moves"""
        if not self.choreography_moves:
            print("📋 No choreography moves found")
            return
            
        print("📋 Choreography Moves:")
        print("-" * 60)
        for name, data in self.choreography_moves.items():
            choreography_type = data['type']
            timestamp = data['timestamp']
            if choreography_type == 'timed_movement':
                duration = data['duration']
                speed = data['speed']
                direction = data['direction']
                print(f"  {name}: {choreography_type} - {direction} for {duration}s at {speed} m/s")
            elif choreography_type == 'spread_align':
                duration = data['duration']
                speed = data['speed']
                print(f"  {name}: {choreography_type} - {duration}s at {speed} m/s")
            elif choreography_type == 'advanced_multi_phase':
                primary_time = data['primary_time']
                secondary_time = data['secondary_time']
                speed = data['speed']
                print(f"  {name}: {choreography_type} - Primary: {primary_time}s, Secondary: {secondary_time}s at {speed} m/s")
            print(f"    Created: {timestamp}")
        print("-" * 60)
        
    def delete_choreography(self, choreography_name):
        """Delete a choreography move"""
        if choreography_name in self.choreography_moves:
            del self.choreography_moves[choreography_name]
            self.save_choreography()
            print(f"🗑️  Deleted choreography: {choreography_name}")
        else:
            print(f"❌ Choreography '{choreography_name}' not found")
        
    def show_position(self):
        """Show current robot position"""
        pos = self.get_position()
        print(f"📍 Current Position:")
        print(f"   X: {pos['x']:.3f} m")
        print(f"   Y: {pos['y']:.3f} m")
        print(f"   θ: {math.degrees(pos['theta']):.1f}°")
        print(f"   Last update: {time.time() - pos['timestamp']:.1f}s ago")
        
    def start_recording(self):
        """Start recording mode"""
        if not self.recording_mode:
            self.recording_mode = True
            self.current_move_sequence = []
            print("🎬 Recording mode STARTED")
            print("   Make your moves, then press SPACE to stop and name the sequence")
        else:
            print("⚠️  Already in recording mode")
            
    def stop_recording(self):
        """Stop recording mode and save the move"""
        if self.recording_mode:
            self.recording_mode = False
            if self.current_move_sequence:
                print(f"📝 Recorded {len(self.current_move_sequence)} moves")
                move_name = input("Enter move name: ").strip()
                if move_name:
                    self.recorded_moves[move_name] = {
                        'sequence': self.current_move_sequence,
                        'timestamp': datetime.now().isoformat(),
                        'speed': self.speed,
                        'turn_speed': self.turn_speed
                    }
                    self.save_recorded_moves()
                    print(f"✅ Saved move: {move_name}")
                else:
                    print("❌ No name provided, move discarded")
            else:
                print("⚠️  No moves recorded")
            self.current_move_sequence = []
        else:
            print("⚠️  Not in recording mode")
            
    def record_move(self, move_type, **kwargs):
        """Record a move with timestamp"""
        if self.recording_mode:
            move_data = {
                'type': move_type,
                'timestamp': time.time(),
                'speed': self.speed,
                'turn_speed': self.turn_speed,
                **kwargs
            }
            self.current_move_sequence.append(move_data)
            
    def play_move(self, move_name):
        """Play a recorded move sequence"""
        if move_name not in self.recorded_moves:
            print(f"❌ Move '{move_name}' not found")
            return
            
        move_data = self.recorded_moves[move_name]
        sequence = move_data['sequence']
        
        print(f"🎭 Playing move: {move_name} ({len(sequence)} actions)")
        
        # Play the sequence
        for i, action in enumerate(sequence):
            print(f"  [{i+1}/{len(sequence)}] {action['type']}")
            
            if action['type'] == 'forward':
                self.send_cmd(linear_x=action['speed'])
            elif action['type'] == 'backward':
                self.send_cmd(linear_x=-action['speed'])
            elif action['type'] == 'left':
                self.send_cmd(linear_y=action['speed'])
            elif action['type'] == 'right':
                self.send_cmd(linear_y=-action['speed'])
            elif action['type'] == 'rotate_left':
                self.send_cmd(angular_z=action['turn_speed'])
            elif action['type'] == 'rotate_right':
                self.send_cmd(angular_z=-action['turn_speed'])
            elif action['type'] == 'diagonal_fl':
                self.send_cmd(linear_x=action['speed'], linear_y=action['speed'])
            elif action['type'] == 'diagonal_fr':
                self.send_cmd(linear_x=action['speed'], linear_y=-action['speed'])
            elif action['type'] == 'stop':
                self.send_cmd()
                
            # Wait for the duration (if specified) or a short delay
            duration = action.get('duration', 0.1)
            time.sleep(duration)
            
        # Stop at the end
        self.send_cmd()
        print(f"✅ Finished playing: {move_name}")
        
    def list_moves(self):
        """List all recorded moves"""
        if not self.recorded_moves:
            print("📋 No recorded moves found")
            return
            
        print("📋 Recorded Moves:")
        print("-" * 50)
        for name, data in self.recorded_moves.items():
            sequence_length = len(data['sequence'])
            timestamp = data['timestamp']
            print(f"  {name}: {sequence_length} actions ({timestamp})")
        print("-" * 50)
        
    def delete_move(self, move_name):
        """Delete a recorded move"""
        if move_name in self.recorded_moves:
            del self.recorded_moves[move_name]
            self.save_recorded_moves()
            print(f"🗑️  Deleted move: {move_name}")
        else:
            print(f"❌ Move '{move_name}' not found")
        
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
                    self.record_move('forward', speed=self.speed)
                    
                elif key == 's':
                    print(f"⬇️  Backward ({self.speed:.2f})")
                    self.send_cmd(linear_x=-self.speed)
                    self.record_move('backward', speed=self.speed)
                    
                elif key == 'a':
                    print(f"⬅️  Left ({self.speed:.2f})")
                    self.send_cmd(linear_y=self.speed)
                    self.record_move('left', speed=self.speed)
                    
                elif key == 'd':
                    print(f"➡️  Right ({self.speed:.2f})")
                    self.send_cmd(linear_y=-self.speed)
                    self.record_move('right', speed=self.speed)
                    
                elif key == 'q':
                    print(f"🔄 Rotate Left ({self.turn_speed:.2f})")
                    self.send_cmd(angular_z=self.turn_speed)
                    self.record_move('rotate_left', turn_speed=self.turn_speed)
                    
                elif key == 'e':
                    print(f"🔃 Rotate Right ({self.turn_speed:.2f})")
                    self.send_cmd(angular_z=-self.turn_speed)
                    self.record_move('rotate_right', turn_speed=self.turn_speed)
                    
                elif key == 'z':
                    print(f"↖️  Diagonal Forward-Left")
                    self.send_cmd(linear_x=self.speed, linear_y=self.speed)
                    self.record_move('diagonal_fl', speed=self.speed)
                    
                elif key == 'c':
                    print(f"↗️  Diagonal Forward-Right")
                    self.send_cmd(linear_x=self.speed, linear_y=-self.speed)
                    self.record_move('diagonal_fr', speed=self.speed)
                    
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
                    self.record_move('stop')
                    
                    # If in recording mode, stop recording after space
                    if self.recording_mode:
                        self.stop_recording()
                    
                elif key == 'R':
                    if self.recording_mode:
                        self.stop_recording()
                    else:
                        self.start_recording()
                        
                elif key == 'p':
                    if self.recorded_moves:
                        print("Available moves:")
                        for name in self.recorded_moves.keys():
                            print(f"  - {name}")
                        move_name = input("Enter move name to play: ").strip()
                        if move_name:
                            self.play_move(move_name)
                    else:
                        print("❌ No recorded moves available")
                        
                elif key == 'l':
                    self.list_moves()
                    
                elif key == 'd':
                    if self.recorded_moves:
                        print("Available moves:")
                        for name in self.recorded_moves.keys():
                            print(f"  - {name}")
                        move_name = input("Enter move name to delete: ").strip()
                        if move_name:
                            self.delete_move(move_name)
                    else:
                        print("❌ No recorded moves available")
                        
                # Choreography controls
                elif key == 'C':
                    if self.choreography_mode:
                        self.choreography_mode = False
                        print("🎭 Choreography mode DISABLED")
                    else:
                        self.choreography_mode = True
                        print("🎭 Choreography mode ENABLED")
                        
                elif key == 't':
                    self.create_timed_movement()
                    
                elif key == 's':
                    self.create_spread_align_movement()
                    
                elif key == 'a':
                    self.create_advanced_multi_phase_movement()
                    
                elif key == 'P':
                    if self.choreography_moves:
                        print("Available choreographies:")
                        for name in self.choreography_moves.keys():
                            print(f"  - {name}")
                        choreography_name = input("Enter choreography name to play: ").strip()
                        if choreography_name:
                            self.play_choreography(choreography_name)
                    else:
                        print("❌ No choreographies available")
                        
                elif key == 'L':
                    self.list_choreography()
                    
                elif key == 'D':
                    if self.choreography_moves:
                        print("Available choreographies:")
                        for name in self.choreography_moves.keys():
                            print(f"  - {name}")
                        choreography_name = input("Enter choreography name to delete: ").strip()
                        if choreography_name:
                            self.delete_choreography(choreography_name)
                    else:
                        print("❌ No choreographies available")
                        
                elif key == 'i':
                    self.show_position()
                    
                elif key == '0':
                    self.reset_position()
                    
                elif key == 'x' or key == '\x03':  # x or Ctrl+C
                    print("👋 Exiting...")
                    break
                    
                else:
                    if key:  # Only print for non-empty keys
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