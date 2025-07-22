import rclpy
import time
from arm_controller import ArmController
from robot_mover import RobotMover

def main(args=None):
    rclpy.init(args=args)
    arm_node = ArmController()
    wheel_node = RobotMover()

    try:
        toggle = True
        while rclpy.ok():
            if toggle:
                arm_node.move_arm(duration_ms=1500, joint4=300, joint5=250, gripper=300)
                wheel_node.move(0.1, 0.3)
            else:
                arm_node.move_arm(duration_ms=1500, joint4=700, joint5=750, gripper=750)
                wheel_node.move(0.1, -0.3)

            toggle = not toggle

            rclpy.spin_once(arm_node, timeout_sec=0.1)
            rclpy.spin_once(wheel_node, timeout_sec=0.1)
            time.sleep(2)

        wheel_node.stop()

    except KeyboardInterrupt:
        print("Stopped by user")
        wheel_node.stop()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

