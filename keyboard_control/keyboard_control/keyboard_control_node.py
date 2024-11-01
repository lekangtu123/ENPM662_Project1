#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import sys
import select
import tty
import termios

# Define key codes
LIN_VEL_STEP_SIZE = 5.0
ANG_VEL_STEP_SIZE = 0.5
MAX_LINEAR_VEL = 15.0  # Define the maximum linear velocity
MAX_ANGULAR_VEL = 0.5  # Define the maximum angular velocity

class KeyboardControlNode(Node):

    def __init__(self):
        super().__init__('keyboard_control_node')

        # Publishers for controlling the robot
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)

        # Setup for reading from the keyboard
        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_keyboard_control(self):
        self.msg = """
        Control Your Car!
        ---------------------------
        Moving around:
            w
        a    s    d

        q : force stop

        Esc to quit
        """

        self.get_logger().info(self.msg)
        joint_positions = Float64MultiArray()
        wheel_velocities = Float64MultiArray()
        linear_vel = 0.0
        steer_angle = 0.0

        while True:
            key = self.getKey()
            if key is not None:
                if key == '\x1b':  # Escape key
                    break
                elif key == 'q':  # Quit
                    linear_vel = 0.0
                    steer_angle = 0.0
                elif key == 'w':  # Forward
                    linear_vel = abs(LIN_VEL_STEP_SIZE)
                elif key == 's':  # Reverse
                    linear_vel = -abs(LIN_VEL_STEP_SIZE)
                elif key == 'd':  # Right
                    steer_angle -= ANG_VEL_STEP_SIZE
                elif key == 'a':  # Left
                    steer_angle += ANG_VEL_STEP_SIZE

                # Limit linear and angular velocities
                linear_vel = max(min(linear_vel, MAX_LINEAR_VEL), -MAX_LINEAR_VEL)
                steer_angle = max(min(steer_angle, MAX_ANGULAR_VEL), -MAX_ANGULAR_VEL)

                print("Steer Angle:", steer_angle)
                print("Linear Velocity:", linear_vel)

                # Publish the wheel velocities and joint positions
                wheel_velocities.data = [linear_vel,-linear_vel,linear_vel]
                joint_positions.data = [steer_angle,steer_angle]

                self.joint_position_pub.publish(joint_positions)
                self.wheel_velocities_pub.publish(wheel_velocities)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    try:
        node.run_keyboard_control()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
