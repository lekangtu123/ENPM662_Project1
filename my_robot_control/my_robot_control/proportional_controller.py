import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from math import atan2, sqrt, cos, sin, atan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from time import sleep
import matplotlib.pyplot as plt


def get_yaw_from_quaternion(qx, qy, qz, qw):
    """
    Calculate yaw from quaternion components.
    """
    # Quaternion to Euler conversion
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = atan2(siny_cosp, cosy_cosp)
    return yaw


class ProportionalControlNode(Node):

    def __init__(self):
        super().__init__('proportional_control_node')

        # Define QoS profile for subscriptions
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        # Create publishers for joint positions, wheel velocities, and joint states
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Create subscription to IMU data
        self.imu_subscription = self.create_subscription(Imu, '/imu_plugin/out', self.control, qos_profile)

        # Define goal coordinates
        self.goalx = 1000.0
        self.goaly = 1000.0

        # Initialize robot's position
        self.posx = 0.0
        self.posy = 0.0

        # Initialize error and linear velocity values for plotting
        self.error_values = []
        self.linear_vel_val = []

    def control(self, imu_msg):
        # Define timestep
        tstep = 0.1

        # Proportional gain for angular velocity control
        kp_angular = 0.8  # Reduced gain to avoid oscillations
        angular_dead_zone = 0.05  # Dead zone to avoid continuous small adjustments

        # Extract quaternion components from IMU message
        qx = imu_msg.orientation.x
        qy = imu_msg.orientation.y
        qz = imu_msg.orientation.z
        qw = imu_msg.orientation.w

        # Calculate real angle (yaw) using quaternion data
        real_ang = get_yaw_from_quaternion(qx, qy, qz, qw)

        # Calculate ideal angle based on target position
        ideal_ang = atan2((self.goaly - self.posy), (self.goalx - self.posx))

        # Calculate steering angle as the difference between the ideal and real angles
        angle_diff = ideal_ang - real_ang

        # Normalize the angle difference to the range [-pi, pi]
        while angle_diff > 3.1416:
            angle_diff -= 2 * 3.1416
        while angle_diff < -3.1416:
            angle_diff += 2 * 3.1416

        # Apply proportional control to calculate the steering angle
        steer_angle = kp_angular * angle_diff

        # Limit steering angle to a reasonable range
        if steer_angle > 1.0:
            steer_angle = 1.0
        elif steer_angle < -1.0:
            steer_angle = -1.0

        # Calculate error between current and target positions
        error = sqrt((self.goalx - self.posx) ** 2 + (self.goaly - self.posy) ** 2)

        # Log real and ideal angles
        self.get_logger().info("[ real angle, ideal angle ] : [ %s, %s ]" % (real_ang, ideal_ang))

        # Check if the robot has reached the target position
        if error < 0.5 or (self.posx >= 1000.0 and self.posy >= 1000.0):
            # Stop the robot and plot the graphs
            linear_vel = 0.0
            self.plot_graphs()
            rclpy.shutdown()
            return
        else:
            # Set linear velocity based on error
            linear_vel = min(10.0, error)  # Limit the velocity to a maximum value of 10

        # If the angle difference is small enough, go straight to avoid oscillations
        if abs(angle_diff) < angular_dead_zone:
            steer_angle = 0.0

        # Update robot's position based on real angle and linear velocity
        self.posx += cos(ideal_ang) * tstep * linear_vel
        self.posy += sin(ideal_ang) * tstep * linear_vel

        # Log robot's current position
        self.get_logger().info("[ x, y ] : [ %s, %s ]" % (self.posx, self.posy))

        # Log linear velocity
        self.get_logger().info("[ linear velocity ] : [ %s ]" % linear_vel)

        # Prepare joint position and wheel velocity messages
        joint_positions = Float64MultiArray()
        wheel_velocities = Float64MultiArray()

        # Set wheel velocities based on the linear velocity
        wheel_velocities.data = [-linear_vel, -linear_vel, linear_vel]

        # Set joint positions based on the steer angle
        joint_positions.data = [steer_angle, steer_angle]

        # Publish joint positions and wheel velocities
        self.joint_position_pub.publish(joint_positions)
        self.wheel_velocities_pub.publish(wheel_velocities)

        # Store error and linear velocity values for plotting
        self.error_values.append(error)
        self.linear_vel_val.append(linear_vel)

    def plot_graphs(self):
        # Plot error and linear velocity over time
        plt.figure(figsize=(10, 5))
        plt.plot(self.error_values, label='Error')
        plt.plot(self.linear_vel_val, label='Linear Velocity')
        plt.xlabel('Time Step')
        plt.ylabel('Values')
        plt.legend()
        plt.grid()
        plt.title('Error and Linear Velocity Over Time')
        plt.show()


def main(args=None):
    rclpy.init(args=args)

    proportional_control_node = ProportionalControlNode()

    try:
        rclpy.spin(proportional_control_node)
    except KeyboardInterrupt:
        proportional_control_node.get_logger().info('Shutting down proportional control node.')
    finally:
        proportional_control_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
