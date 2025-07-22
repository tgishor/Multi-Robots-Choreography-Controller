#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import time
from flask import Flask, render_template, request, jsonify

class WebController(Node):
    def __init__(self):
        super().__init__('web_controller')
        self.publisher_ = self.create_publisher(Twist, 'controller/cmd_vel', 10)
        self.speed = 0.1
        self.is_moving = False
        
    def send_command(self, linear_x=0, linear_y=0, angular_z=0, duration=2.0):
        """Send movement command for specified duration"""
        self.is_moving = True
        
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.linear.y = float(linear_y)
        twist.angular.z = float(angular_z)
        
        # Send commands at 10Hz for duration
        end_time = time.time() + duration
        while time.time() < end_time and self.is_moving:
            self.publisher_.publish(twist)
            time.sleep(0.1)
        
        # Stop
        stop_twist = Twist()
        for _ in range(5):
            self.publisher_.publish(stop_twist)
            time.sleep(0.1)
            
        self.is_moving = False
        
    def emergency_stop(self):
        """Emergency stop"""
        self.is_moving = False
        stop_twist = Twist()
        for _ in range(10):
            self.publisher_.publish(stop_twist)
            time.sleep(0.05)

# Global robot controller
robot = None

# Flask web app
app = Flask(__name__)



@app.route('/')
def index():
    return render_template('index.html')

@app.route('/move', methods=['POST'])
def move():
    global robot
    data = request.json
    direction = data['direction']
    speed = data['speed']
    
    movements = {
        'forward': (speed, 0, 0),
        'backward': (-speed, 0, 0),
        'left': (0, speed, 0),
        'right': (0, -speed, 0),
        'rotate_left': (0, 0, speed * 2),
        'rotate_right': (0, 0, -speed * 2),
        'diag_fl': (speed, speed, 0),
        'diag_fr': (speed, -speed, 0),
        'circle': (speed, 0, speed)
    }
    
    if direction in movements:
        linear_x, linear_y, angular_z = movements[direction]
        # Run in background thread
        threading.Thread(
            target=robot.send_command,
            args=(linear_x, linear_y, angular_z, 2.0),
            daemon=True
        ).start()
        return jsonify({'status': f'{direction.title()} completed'})
    
    return jsonify({'status': 'Unknown direction'}), 400

@app.route('/stop', methods=['POST'])
def stop():
    global robot
    robot.emergency_stop()
    return jsonify({'status': 'Stopped'})

def main():
    global robot
    
    print("Starting Web-Based Mecanum Controller...")
    
    # Initialize ROS2
    rclpy.init()
    robot = WebController()
    
    # Start ROS2 spinning in background
    ros_thread = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    ros_thread.start()
    
    print("ROS2 initialized")
    print("Starting web server...")
    print("Open browser and go to: http://localhost:5000")
    print("Use the web interface to control your robot!")
    
    try:
        app.run(host='0.0.0.0', port=5000, debug=False)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()