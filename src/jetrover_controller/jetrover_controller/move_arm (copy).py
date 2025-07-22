import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import SetBusServoState, BusServoState
import time

class ArmMover(Node):
    def __init__(self):
        super().__init__('arm_mover')
        self.pub = self.create_publisher(SetBusServoState, '/ros_robot_controller/bus_servo/set_state', 10)

    def move_joint(self, servo_id, angle, duration_ms):
        # Create a single BusServoState entry
        servo_state = BusServoState()
        servo_state.target_id = [servo_id]      # Which servo ID to move
        servo_state.position = [angle]          # Desired angle (0–1000)
        servo_state.enable_torque = [1]         # Enable torque (important!)
        servo_state.stop = [0]                 # Make sure 'stop' is 0 if required

        # Build the SetBusServoState message
        msg = SetBusServoState()
        msg.state.append(servo_state)
        # Duration in seconds
        msg.duration = duration_ms / 1000.0

        self.pub.publish(msg)
        self.get_logger().info(f"Moving servo {servo_id} to angle {angle} over {duration_ms} ms")
        time.sleep(duration_ms / 1000.0)  # Wait for the movement

def main(args=None):
    rclpy.init(args=args)
    node = ArmMover()

    # EXAMPLE: Move servo 1 to angle=500 in 1 second
    node.move_joint(1, 700, 2000)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

