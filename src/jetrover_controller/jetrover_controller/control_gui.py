#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import tkinter as tk
from tkinter import ttk, messagebox
import threading
from datetime import datetime

class MecanumRobotGUI(Node):
    def __init__(self):
        super().__init__('mecanum_robot_gui')
        
        # ROS2 Setup
        self.publisher_ = self.create_publisher(Twist, 'controller/cmd_vel', 10)
        time.sleep(0.5)  # Give time to connect
        
        # Movement state
        self.is_moving = False
        self.current_movement = "Stopped"
        self.movement_thread = None
        
        # Create GUI
        self.setup_gui()
        
        self.get_logger().info("🤖 Mecanum Robot GUI Ready!")
        
    def setup_gui(self):
        """Create the main GUI window"""
        self.root = tk.Tk()
        self.root.title("🤖 Mecanum Robot Controller")
        self.root.geometry("800x700")
        self.root.configure(bg='#2c3e50')
        
        # Configure styles
        style = ttk.Style()
        style.theme_use('clam')
        
        # Main container
        main_frame = tk.Frame(self.root, bg='#2c3e50', padx=20, pady=20)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Title
        title_label = tk.Label(main_frame, text="🤖 MECANUM ROBOT CONTROLLER", 
                              font=('Arial', 20, 'bold'), fg='#ecf0f1', bg='#2c3e50')
        title_label.pack(pady=(0, 20))
        
        # Status Frame
        self.create_status_frame(main_frame)
        
        # Control Panel
        control_frame = tk.Frame(main_frame, bg='#34495e', relief=tk.RAISED, bd=2)
        control_frame.pack(fill=tk.BOTH, expand=True, pady=10)
        
        # Speed and Duration Controls
        self.create_control_panel(control_frame)
        
        # Movement Buttons
        self.create_movement_buttons(control_frame)
        
        # Emergency Stop
        self.create_emergency_controls(main_frame)
        
        # Status update timer
        self.update_status()
        
    def create_status_frame(self, parent):
        """Create status display frame"""
        status_frame = tk.LabelFrame(parent, text="Robot Status", font=('Arial', 12, 'bold'),
                                   fg='#ecf0f1', bg='#34495e', relief=tk.RAISED, bd=2)
        status_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Current movement status
        self.status_label = tk.Label(status_frame, text="Status: Ready", 
                                   font=('Arial', 14, 'bold'), fg='#2ecc71', bg='#34495e')
        self.status_label.pack(pady=5)
        
        # Last command info
        self.command_label = tk.Label(status_frame, text="Last Command: None", 
                                    font=('Arial', 10), fg='#ecf0f1', bg='#34495e')
        self.command_label.pack()
        
        # Timestamp
        self.time_label = tk.Label(status_frame, text="", 
                                 font=('Arial', 9), fg='#bdc3c7', bg='#34495e')
        self.time_label.pack()
        
    def create_control_panel(self, parent):
        """Create speed and duration controls"""
        control_panel = tk.Frame(parent, bg='#34495e')
        control_panel.pack(fill=tk.X, padx=10, pady=10)
        
        # Speed Control
        speed_frame = tk.Frame(control_panel, bg='#34495e')
        speed_frame.pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        tk.Label(speed_frame, text="Speed (m/s):", font=('Arial', 10, 'bold'),
                fg='#ecf0f1', bg='#34495e').pack()
        
        self.speed_var = tk.DoubleVar(value=0.1)
        self.speed_scale = tk.Scale(speed_frame, from_=0.05, to=0.3, resolution=0.05,
                                  orient=tk.HORIZONTAL, variable=self.speed_var,
                                  bg='#34495e', fg='#ecf0f1', highlightbackground='#34495e')
        self.speed_scale.pack(fill=tk.X, padx=5)
        
        # Duration Control
        duration_frame = tk.Frame(control_panel, bg='#34495e')
        duration_frame.pack(side=tk.RIGHT, fill=tk.X, expand=True)
        
        tk.Label(duration_frame, text="Duration (seconds):", font=('Arial', 10, 'bold'),
                fg='#ecf0f1', bg='#34495e').pack()
        
        self.duration_var = tk.DoubleVar(value=2.0)
        self.duration_scale = tk.Scale(duration_frame, from_=0.5, to=5.0, resolution=0.5,
                                     orient=tk.HORIZONTAL, variable=self.duration_var,
                                     bg='#34495e', fg='#ecf0f1', highlightbackground='#34495e')
        self.duration_scale.pack(fill=tk.X, padx=5)
        
    def create_movement_buttons(self, parent):
        """Create all movement control buttons"""
        button_frame = tk.Frame(parent, bg='#34495e')
        button_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Basic Movement Buttons (Grid Layout)
        basic_frame = tk.LabelFrame(button_frame, text="Basic Movements", 
                                  font=('Arial', 12, 'bold'), fg='#ecf0f1', bg='#34495e')
        basic_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        
        # Forward button (top center)
        self.create_button(basic_frame, "🔺\nFORWARD", self.move_forward, 
                         '#3498db', row=0, col=1)
        
        # Left and Right buttons (middle row)
        self.create_button(basic_frame, "◀️\nLEFT", self.move_left, 
                         '#e74c3c', row=1, col=0)
        self.create_button(basic_frame, "▶️\nRIGHT", self.move_right, 
                         '#e74c3c', row=1, col=2)
        
        # Backward button (bottom center)
        self.create_button(basic_frame, "🔻\nBACKWARD", self.move_backward, 
                         '#3498db', row=2, col=1)
        
        # Advanced Movement Buttons
        advanced_frame = tk.LabelFrame(button_frame, text="Advanced Movements", 
                                     font=('Arial', 12, 'bold'), fg='#ecf0f1', bg='#34495e')
        advanced_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))
        
        # Rotation buttons
        self.create_button(advanced_frame, "🔄\nROTATE LEFT", self.rotate_left, 
                         '#f39c12', row=0, col=0)
        self.create_button(advanced_frame, "🔃\nROTATE RIGHT", self.rotate_right, 
                         '#f39c12', row=0, col=1)
        
        # Diagonal buttons
        self.create_button(advanced_frame, "↗️\nFORWARD-RIGHT", self.diagonal_forward_right, 
                         '#9b59b6', row=1, col=0)
        self.create_button(advanced_frame, "↖️\nFORWARD-LEFT", self.diagonal_forward_left, 
                         '#9b59b6', row=1, col=1)
        
        # Special movements
        self.create_button(advanced_frame, "🌀\nCIRCLE", self.move_circle, 
                         '#1abc9c', row=2, col=0)
        self.create_button(advanced_frame, "🎯\nCUSTOM", self.custom_movement, 
                         '#34495e', row=2, col=1)
        
    def create_button(self, parent, text, command, color, row, col):
        """Create a styled movement button"""
        btn = tk.Button(parent, text=text, command=command,
                       font=('Arial', 10, 'bold'), fg='white', bg=color,
                       relief=tk.RAISED, bd=3, padx=10, pady=10,
                       activebackground='#ecf0f1', activeforeground=color)
        btn.grid(row=row, column=col, padx=5, pady=5, sticky='nsew')
        
        # Configure grid weights for responsive design
        parent.grid_rowconfigure(row, weight=1)
        parent.grid_columnconfigure(col, weight=1)
        
        return btn
        
    def create_emergency_controls(self, parent):
        """Create emergency stop and exit controls"""
        emergency_frame = tk.Frame(parent, bg='#2c3e50')
        emergency_frame.pack(fill=tk.X, pady=(10, 0))
        
        # Emergency Stop
        stop_btn = tk.Button(emergency_frame, text="🛑 EMERGENCY STOP", 
                           command=self.emergency_stop, font=('Arial', 14, 'bold'),
                           fg='white', bg='#e74c3c', relief=tk.RAISED, bd=4,
                           padx=20, pady=10)
        stop_btn.pack(side=tk.LEFT, padx=(0, 10))
        
        # Exit button
        exit_btn = tk.Button(emergency_frame, text="❌ EXIT", 
                           command=self.exit_application, font=('Arial', 12, 'bold'),
                           fg='white', bg='#95a5a6', relief=tk.RAISED, bd=3,
                           padx=15, pady=8)
        exit_btn.pack(side=tk.RIGHT)
        
    def move_robot(self, linear_x=0.0, linear_y=0.0, angular_z=0.0, duration=None):
        """Core movement function with GUI integration"""
        if self.is_moving:
            messagebox.showwarning("Robot Busy", "Robot is currently moving. Please wait or use Emergency Stop.")
            return
            
        if duration is None:
            duration = self.duration_var.get()
            
        # Update status
        self.is_moving = True
        movement_type = self.get_movement_type(linear_x, linear_y, angular_z)
        self.current_movement = movement_type
        
        # Start movement in separate thread
        self.movement_thread = threading.Thread(
            target=self._execute_movement, 
            args=(linear_x, linear_y, angular_z, duration, movement_type)
        )
        self.movement_thread.daemon = True
        self.movement_thread.start()
        
    def _execute_movement(self, linear_x, linear_y, angular_z, duration, movement_type):
        """Execute the actual movement (runs in separate thread)"""
        try:
            twist = Twist()
            twist.linear.x = linear_x
            twist.linear.y = linear_y
            twist.angular.z = angular_z
            
            self.get_logger().info(f"🔄 {movement_type}: x={linear_x:.2f}, y={linear_y:.2f}, z={angular_z:.2f} for {duration:.1f}s")
            
            # Send commands continuously at 10Hz
            rate = 10  # Hz
            total_iterations = int(duration * rate)
            
            for i in range(total_iterations):
                if not self.is_moving:  # Check for stop request
                    break
                self.publisher_.publish(twist)
                time.sleep(1.0 / rate)
            
            # Stop the robot
            self.stop_robot()
            
        except Exception as e:
            self.get_logger().error(f"Movement error: {e}")
        finally:
            self.is_moving = False
            self.current_movement = "Stopped"
            
    def stop_robot(self):
        """Stop the robot"""
        stop_twist = Twist()  # All zeros
        for i in range(10):
            self.publisher_.publish(stop_twist)
            time.sleep(0.05)
            
    def get_movement_type(self, x, y, z):
        """Get human-readable movement type"""
        if x > 0 and y == 0 and z == 0:
            return "Moving Forward"
        elif x < 0 and y == 0 and z == 0:
            return "Moving Backward"
        elif x == 0 and y > 0 and z == 0:
            return "Moving Left"
        elif x == 0 and y < 0 and z == 0:
            return "Moving Right"
        elif x == 0 and y == 0 and z > 0:
            return "Rotating Left"
        elif x == 0 and y == 0 and z < 0:
            return "Rotating Right"
        elif x > 0 and y > 0:
            return "Diagonal Forward-Left"
        elif x > 0 and y < 0:
            return "Diagonal Forward-Right"
        elif x > 0 and z != 0:
            return "Moving in Circle"
        else:
            return "Custom Movement"
    
    # Movement command methods
    def move_forward(self):
        self.move_robot(linear_x=self.speed_var.get())
        
    def move_backward(self):
        self.move_robot(linear_x=-self.speed_var.get())
        
    def move_left(self):
        self.move_robot(linear_y=self.speed_var.get())
        
    def move_right(self):
        self.move_robot(linear_y=-self.speed_var.get())
        
    def rotate_left(self):
        self.move_robot(angular_z=self.speed_var.get() * 2)  # Faster rotation
        
    def rotate_right(self):
        self.move_robot(angular_z=-self.speed_var.get() * 2)
        
    def diagonal_forward_left(self):
        speed = self.speed_var.get()
        self.move_robot(linear_x=speed, linear_y=speed)
        
    def diagonal_forward_right(self):
        speed = self.speed_var.get()
        self.move_robot(linear_x=speed, linear_y=-speed)
        
    def move_circle(self):
        speed = self.speed_var.get()
        self.move_robot(linear_x=speed, angular_z=speed)
        
    def custom_movement(self):
        """Open custom movement dialog"""
        self.open_custom_dialog()
        
    def open_custom_dialog(self):
        """Custom movement input dialog"""
        dialog = tk.Toplevel(self.root)
        dialog.title("Custom Movement")
        dialog.geometry("400x300")
        dialog.configure(bg='#34495e')
        dialog.grab_set()
        
        # Input fields
        tk.Label(dialog, text="Custom Movement Parameters", font=('Arial', 14, 'bold'),
                fg='#ecf0f1', bg='#34495e').pack(pady=10)
        
        # Linear X
        tk.Label(dialog, text="Linear X (forward/backward):", fg='#ecf0f1', bg='#34495e').pack()
        linear_x_var = tk.DoubleVar()
        tk.Scale(dialog, from_=-0.3, to=0.3, resolution=0.05, orient=tk.HORIZONTAL,
                variable=linear_x_var, bg='#34495e', fg='#ecf0f1').pack(fill=tk.X, padx=20)
        
        # Linear Y
        tk.Label(dialog, text="Linear Y (left/right):", fg='#ecf0f1', bg='#34495e').pack()
        linear_y_var = tk.DoubleVar()
        tk.Scale(dialog, from_=-0.3, to=0.3, resolution=0.05, orient=tk.HORIZONTAL,
                variable=linear_y_var, bg='#34495e', fg='#ecf0f1').pack(fill=tk.X, padx=20)
        
        # Angular Z
        tk.Label(dialog, text="Angular Z (rotation):", fg='#ecf0f1', bg='#34495e').pack()
        angular_z_var = tk.DoubleVar()
        tk.Scale(dialog, from_=-0.5, to=0.5, resolution=0.05, orient=tk.HORIZONTAL,
                variable=angular_z_var, bg='#34495e', fg='#ecf0f1').pack(fill=tk.X, padx=20)
        
        # Execute button
        execute_btn = tk.Button(dialog, text="Execute Movement", 
                              command=lambda: [
                                  self.move_robot(linear_x_var.get(), linear_y_var.get(), angular_z_var.get()),
                                  dialog.destroy()
                              ],
                              bg='#27ae60', fg='white', font=('Arial', 12, 'bold'))
        execute_btn.pack(pady=20)
        
    def emergency_stop(self):
        """Emergency stop function"""
        self.is_moving = False
        self.current_movement = "EMERGENCY STOPPED"
        self.stop_robot()
        messagebox.showinfo("Emergency Stop", "Robot has been stopped!")
        
    def exit_application(self):
        """Exit the application safely"""
        if self.is_moving:
            self.emergency_stop()
        
        self.root.quit()
        self.root.destroy()
        
    def update_status(self):
        """Update status display"""
        # Update status text
        if self.is_moving:
            self.status_label.config(text=f"Status: {self.current_movement}", fg='#f39c12')
        else:
            self.status_label.config(text="Status: Ready", fg='#2ecc71')
            
        # Update timestamp
        current_time = datetime.now().strftime("%H:%M:%S")
        self.time_label.config(text=f"Last Update: {current_time}")
        
        # Schedule next update
        self.root.after(500, self.update_status)  # Update every 500ms
        
    def run(self):
        """Run the GUI application"""
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            self.exit_application()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        # Create and run GUI
        robot_gui = MecanumRobotGUI()
        
        # Run ROS2 spinning in a separate thread
        ros_thread = threading.Thread(target=rclpy.spin, args=(robot_gui,), daemon=True)
        ros_thread.start()
        
        # Run GUI in main thread
        robot_gui.run()
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()