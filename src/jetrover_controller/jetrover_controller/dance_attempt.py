#!/usr/bin/env python3
import os
import time
import threading
import subprocess
import argparse
import random
import rclpy
from rclpy.node import Node
from servo_controller_msgs.msg import ServosPosition, ServoPosition, ServoStateList
from geometry_msgs.msg import Twist

import librosa
import numpy as np

default_audio = '/home/ubuntu/axy_proj/src/jetrover_controller/jetrover_controller/music.mp3'

class DanceNode(Node):
    def __init__(self, audio_path, audio_player='mpg123', beats_per_move=4):
        super().__init__('dance_node')
        self.audio_path = audio_path
        self.audio_player = audio_player
        self.beats_per_move = beats_per_move

        if not os.path.isfile(self.audio_path):
            self.get_logger().error(f"Audio file not found: {self.audio_path}")
            raise FileNotFoundError(self.audio_path)

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
        self.last_move_direction = -1  # for alternating forward/backward

        self.analyze_audio_beats()
        threading.Thread(target=self.wait_and_dance, daemon=True).start()

    def servo_state_cb(self, msg: ServoStateList):
        for s in msg.servo_state:
            self.current_pose[s.id] = s.position
        self.pose_received = True

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
        threading.Thread(target=self.play_audio, daemon=True).start()
        time.sleep(1.0)
        start_time = time.time()

        for i, (s, e, label, amp) in enumerate(self.sections):
            wait = (start_time + s) - time.time()
            if wait > 0:
                time.sleep(wait)

            dur = e - s
            self.get_logger().info(f"Beat {i+1}/{len(self.sections)}: {label}, dur={dur:.2f}s, amp={amp:.1f}")
            targets = self.generate_motion(label, amp, home)
            self.pub_servo_move(dur, targets)

            # Front wheel wiggle every 4 beats
            if i % 4 == 0:
                steering_msg = ServosPosition()
                steering_msg.position_unit = 'pulse'
                steering_msg.duration = 0.3
                steering_angle = random.choice([250, 650])
                sp = ServoPosition()
                sp.id = 9
                sp.position = float(steering_angle)
                steering_msg.position.append(sp)
                self.servo_pub.publish(steering_msg)
                time.sleep(0.4)
                center_msg = ServosPosition()
                center_msg.position_unit = 'pulse'
                center_msg.duration = 0.2
                sp_center = ServoPosition()
                sp_center.id = 9
                sp_center.position = 500.0
                center_msg.position.append(sp_center)
                self.servo_pub.publish(center_msg)

            # Move every 8 beats if amp is high
            if i % 8 == 0 and amp > 200:
                twist = Twist()
                self.last_move_direction *= -1
                twist.linear.x = 0.001 * amp * self.last_move_direction
                direction_str = "forward" if self.last_move_direction == 1 else "backward"
                self.get_logger().info(f"Beat {i+1}: Moving {direction_str} with amp {amp:.1f}")
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.3)
                self.cmd_vel_pub.publish(Twist())
                time.sleep(0.1)

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
    parser = argparse.ArgumentParser(description="JetRover music-synced beat dance engine")
    parser.add_argument('--audio', default=default_audio, help="Path to audio file")
    parser.add_argument('--player', default='mpg123', choices=['mpg123', 'mpg321', 'mplayer', 'aplay'], help="Audio player")
    parser.add_argument('--beats-per-move', type=int, default=4, help="Beats per motion segment")
    args, _ = parser.parse_known_args()

    rclpy.init()
    try:
        node = DanceNode(args.audio, args.player, args.beats_per_move)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
