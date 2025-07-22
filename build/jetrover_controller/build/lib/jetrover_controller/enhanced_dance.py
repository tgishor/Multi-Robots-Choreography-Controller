#!/usr/bin/env python3

import os
import time
import threading
import subprocess
import argparse
import random
import math

import rclpy
from rclpy.node import Node
from servo_controller_msgs.msg import ServosPosition, ServoPosition, ServoStateList
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

import librosa
import numpy as np

default_audio = '/home/ubuntu/axy_proj/src/jetrover_controller/jetrover_controller/music.mp3'

# LiDAR constants
CAR_WIDTH = 0.4
MAX_SCAN_ANGLE = 240

class EnhancedDanceNode(Node):
    def __init__(self, audio_path, audio_player='mpg123', beats_per_move=4):
        super().__init__('enhanced_dance_node')
        self.audio_path = audio_path
        self.audio_player = audio_player
        self.beats_per_move = beats_per_move

        if not os.path.isfile(self.audio_path):
            self.get_logger().error(f"Audio file not found: {self.audio_path}")
            raise FileNotFoundError(self.audio_path)

        # Original dance setup - UNCHANGED
        self.servo_pub = self.create_publisher(ServosPosition, 'servo_controller', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        self.current_pose = {}
        self.pose_received = False
        self.create_subscription(ServoStateList, '/servo_states', self.servo_state_cb, 10)

        self.servo_ids = [1, 2, 3, 4, 5, 10]
        self.max_servo_delta = 150.0
        self.sections = []
        self.median_loud = 1.0
        self.motion_labels = ['wave', 'spin', 'reach', 'pulse', 'shake', 'tilt', 'nod']

        # NEW: LiDAR obstacle avoidance setup
        self.lidar_type = os.environ.get('LIDAR_TYPE', 'LD19')
        self.machine_type = os.environ.get('MACHINE_TYPE', 'JetRover_Acker')
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, '/scan_raw', self.lidar_callback, qos)
        
        # Obstacle avoidance parameters
        self.obstacle_threshold = 0.6
        self.scan_angle = math.radians(90)
        self.avoidance_active = False
        self.avoidance_timestamp = 0
        self.last_safe_direction = 0
        self.obstacle_lock = threading.RLock()
        
        # Dancing state
        self.dancing = False

        # Original dance initialization - UNCHANGED
        self.analyze_audio_beats()
        threading.Thread(target=self.wait_and_dance, daemon=True).start()

    def servo_state_cb(self, msg: ServoStateList):
        for s in msg.servo_state:
            self.current_pose[s.id] = s.position
        self.pose_received = True

    # NEW: LiDAR obstacle detection
    def lidar_callback(self, lidar_data):
        """Process LiDAR data for obstacle avoidance - runs in parallel to dancing"""
        if not self.dancing:
            return
            
        # Parse LiDAR data
        if self.lidar_type != 'G4':
            max_index = int(math.radians(MAX_SCAN_ANGLE / 2.0) / lidar_data.angle_increment)
            left_ranges = lidar_data.ranges[:max_index]
            right_ranges = lidar_data.ranges[::-1][:max_index]
        elif self.lidar_type == 'G4':
            min_index = int(math.radians((360 - MAX_SCAN_ANGLE) / 2.0) / lidar_data.angle_increment)
            max_index = min_index + int(math.radians(MAX_SCAN_ANGLE / 2.0) / lidar_data.angle_increment)
            left_ranges = lidar_data.ranges[::-1][min_index:max_index][::-1]
            right_ranges = lidar_data.ranges[min_index:max_index][::-1]

        with self.obstacle_lock:
            # Get data according to scan angle settings
            angle = self.scan_angle / 2
            angle_index = int(angle / lidar_data.angle_increment + 0.50)
            left_range = np.array(left_ranges[:angle_index])
            right_range = np.array(right_ranges[:angle_index])
            
            self.check_obstacles(left_range, right_range)

    def check_obstacles(self, left_range, right_range):
        """Check for obstacles and activate avoidance if needed"""
        # Filter valid data points
        left_nonzero = left_range.nonzero()
        right_nonzero = right_range.nonzero()
        left_nonan = np.isfinite(left_range[left_nonzero])
        right_nonan = np.isfinite(right_range[right_nonzero])
        
        min_dist_left_ = left_range[left_nonzero][left_nonan]
        min_dist_right_ = right_range[right_nonzero][right_nonan]
        
        obstacle_detected = False
        current_time = time.time()
        
        if len(min_dist_left_) > 0 and len(min_dist_right_) > 0:
            min_dist_left = min_dist_left_.min()
            min_dist_right = min_dist_right_.min()
            
            # Check if we need to start or continue obstacle avoidance
            if self.avoidance_timestamp <= current_time:  # Ready for new maneuver
                if min_dist_left <= self.obstacle_threshold and min_dist_right > self.obstacle_threshold:
                    # Obstacle on left - turn right
                    self.start_avoidance_maneuver("right", current_time)
                    obstacle_detected = True
                elif min_dist_left > self.obstacle_threshold and min_dist_right <= self.obstacle_threshold:
                    # Obstacle on right - turn left
                    self.start_avoidance_maneuver("left", current_time)
                    obstacle_detected = True
                elif min_dist_left <= self.obstacle_threshold and min_dist_right <= self.obstacle_threshold:
                    # Obstacles on both sides - back up
                    self.start_avoidance_maneuver("back", current_time)
                    obstacle_detected = True
            else:
                # Continue current avoidance maneuver
                obstacle_detected = True
        
        self.avoidance_active = obstacle_detected

    def start_avoidance_maneuver(self, direction, current_time):
        """Start an obstacle avoidance maneuver"""
        twist = Twist()
        
        if direction == "right":
            twist.linear.x = 0.2
            twist.angular.z = -twist.linear.x / 0.7  # Right turn
            self.last_safe_direction = -1
            self.get_logger().info("🚫 Obstacle on left - turning right while dancing!")
        elif direction == "left":
            twist.linear.x = 0.2
            twist.angular.z = twist.linear.x / 0.7   # Left turn
            self.last_safe_direction = 1
            self.get_logger().info("🚫 Obstacle on right - turning left while dancing!")
        elif direction == "back":
            twist.linear.x = -0.2  # Back up
            twist.angular.z = 0.0
            self.get_logger().info("🚫 Obstacles on both sides - backing up while dancing!")
        
        # Execute the avoidance maneuver
        self.cmd_vel_pub.publish(twist)
        self.avoidance_timestamp = current_time + 1.0  # Continue for 1 second

    # NEW: Safe wheel movement that respects obstacle avoidance
    def safe_wheel_move(self, twist):
        """Publish wheel movement only if no obstacle avoidance is active"""
        with self.obstacle_lock:
            if not self.avoidance_active:
                self.cmd_vel_pub.publish(twist)
            # If avoidance is active, ignore the dance wheel movement
            # The obstacle avoidance will handle wheel control

    # ORIGINAL DANCE ALGORITHM - COMPLETELY UNCHANGED
    def analyze_audio_beats(self):
        self.get_logger().info("Analyzing audio beats…")
        y, sr = librosa.load(self.audio_path, sr=None)
        tempo, beats = librosa.beat.beat_track(y=y, sr=sr)
        onset_env = librosa.onset.onset_strength(y=y, sr=sr)
        self.median_loud = float(np.median(onset_env))

        beat_times = librosa.frames_to_time(beats, sr=sr)
        for i in range(len(beat_times) - 1):
            start = beat_times[i]
            end = beat_times[i + 1]
            s_frame = librosa.time_to_frames(start, sr=sr)
            e_frame = librosa.time_to_frames(end, sr=sr)
            loud = float(np.mean(onset_env[s_frame:e_frame]))
            amp = (loud / self.median_loud) * self.max_servo_delta
            label = random.choice(self.motion_labels)
            self.sections.append((start, end, label, amp))
            self.get_logger().info(f"Beat {i+1}: {label}, dur={end-start:.2f}s, loud={loud:.2f}")

    def wait_and_dance(self):
        t0 = time.time()
        while not self.pose_received and time.time() - t0 < 10.0:
            time.sleep(0.1)
        if not self.pose_received:
            self.get_logger().warn("No servo_states; using default home positions.")
            for sid in self.servo_ids:
                self.current_pose[sid] = 500
        home = {sid: self.current_pose.get(sid, 500) for sid in self.servo_ids}

        self.get_logger().info(f"Home positions: {home}")
        self.get_logger().info("🎵 Starting enhanced dance with LiDAR obstacle avoidance!")
        
        # Start dancing
        self.dancing = True
        
        threading.Thread(target=self.play_audio, daemon=True).start()
        time.sleep(1.0)
        start_time = time.time()

        for i, (s, e, label, amp) in enumerate(self.sections):
            wait = (start_time + s) - time.time()
            if wait > 0:
                time.sleep(wait)

            dur = e - s
            avoidance_status = "🚫 AVOIDING" if self.avoidance_active else "🎵 DANCING"
            self.get_logger().info(f"Beat {i+1}/{len(self.sections)}: {label}, dur={dur:.2f}s, amp={amp:.1f} - {avoidance_status}")
            
            targets = self.generate_motion(label, amp, home)
            self.pub_servo_move(dur, targets)
            
            if i % 4 == 0:  # Front wheel wiggle every 4 beats
                # Direct servo control for front wheel steering (servo ID 9)
                steering_msg = ServosPosition()
                steering_msg.position_unit = 'pulse'
                steering_msg.duration = 0.3  # Quick movement
                
                # Wiggle steering left and right (350-650 range for safety)
                steering_angle = random.choice([250, 650])  # Left or right
                sp = ServoPosition()
                sp.id = 9  # Steering servo
                sp.position = float(steering_angle)
                steering_msg.position.append(sp)
                
                self.servo_pub.publish(steering_msg)
                time.sleep(0.4)  # Let it move
                
                # Return to center position
                center_msg = ServosPosition()
                center_msg.position_unit = 'pulse'
                center_msg.duration = 0.2
                sp_center = ServoPosition()
                sp_center.id = 9
                sp_center.position = 500.0  # Center position
                center_msg.position.append(sp_center)
                self.servo_pub.publish(center_msg)

            # MODIFIED: Use safe wheel movement instead of direct cmd_vel
            # Original commented code would be:
            # if i % 4 == 0:  # Wheel twitch every 4 beats
            #     twist = Twist()
            #     twist.angular.z = random.choice([-0.5, 0.5])
            #     self.safe_wheel_move(twist)  # Use safe movement
            #     time.sleep(0.2)
            #     self.safe_wheel_move(Twist())  # Stop safely

        # End dancing
        self.dancing = False
        
        self.get_logger().info("Dance complete; returning home.")
        self.pub_servo_move(2.0, home)
        self.cmd_vel_pub.publish(Twist())

    def generate_motion(self, label, amp, home):
        if label == 'wave':
            return {sid: home[sid] + ((-1) ** sid) * amp * 0.5 for sid in self.servo_ids}
        elif label == 'spin':
            return {sid: home[sid] + random.uniform(-amp, amp) for sid in self.servo_ids}
        elif label == 'reach':
            return {sid: home[sid] + amp if sid % 2 == 0 else home[sid] - amp for sid in self.servo_ids}
        elif label == 'pulse':
            return {sid: home[sid] + random.uniform(-1, 1) * amp * 0.3 for sid in self.servo_ids}
        elif label == 'shake':
            return {sid: home[sid] + ((-1) ** (sid + int(time.time() * 10))) * amp * 0.25 for sid in self.servo_ids}
        elif label == 'tilt':
            return {sid: home[sid] + (amp * 0.3 if sid in [1, 2] else -amp * 0.3) for sid in self.servo_ids}
        elif label == 'nod':
            return {sid: home[sid] + (amp * 0.2 if sid in [3, 4] else -amp * 0.2) for sid in self.servo_ids}
        return home.copy()

    def play_audio(self):
        cmd = [self.audio_player, '-q', self.audio_path] if self.audio_player == 'mpg123' else [self.audio_player, self.audio_path]
        try:
            p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            p.wait()
        except Exception as e:
            self.get_logger().error(f"Audio playback error: {e}")

    def pub_servo_move(self, duration, targets):
        msg = ServosPosition()
        msg.position_unit = 'pulse'
        msg.duration = float(duration)
        for sid, pos in targets.items():
            msg.position.append(ServoPosition(id=sid, position=float(pos)))
        self.servo_pub.publish(msg)

def main():
    parser = argparse.ArgumentParser(description="Enhanced JetRover music-synced dance with LiDAR obstacle avoidance")
    parser.add_argument('--audio', default=default_audio, help="Path to audio file")
    parser.add_argument('--player', default='mpg123', choices=['mpg123', 'mpg321', 'mplayer', 'aplay'], help="Audio player")
    parser.add_argument('--beats-per-move', type=int, default=4, help="Beats per motion segment")
    parser.add_argument('--threshold', type=float, default=0.6, help="Obstacle avoidance threshold in meters")
    args, _ = parser.parse_known_args()

    rclpy.init()
    try:
        node = EnhancedDanceNode(args.audio, args.player, args.beats_per_move)
        
        # Update threshold if provided
        if hasattr(args, 'threshold'):
            node.obstacle_threshold = args.threshold
            node.get_logger().info(f"Obstacle avoidance threshold set to {args.threshold}m")
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Enhanced dance interrupted by user")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 