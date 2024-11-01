import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from math import atan2, sqrt, cos, sin, atan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from time import sleep
import matplotlib.pyplot as plt


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
        self.goalx = 10.0
        self.goaly = 10.0

        # Initialize robot's position
        self.posx = 0.0
        self.posy = 0.0

        # Initialize error and linear velocity values for plotting
        self.error_values = []
        self.linear_vel_val = []

    def control(self, imu_msg):
        # Define timestep
        tstep = 0.1

        # Initialize linear velocity
        linear_vel = 0.0

        # Extract quaternion components from IMU message
        qx = imu_msg.orientation.x
        qy = imu_msg.orientation.y
        qz = imu_msg.orientation.z
        qw = imu_msg.orientation.w

        # Calculate real angle (yaw) using quaternion data
        aav = get_yaw_from_quaternion(qx, qy, qz, qw)
        real_ang = 3.1416 - aav

        # Calculate ideal angle based on target position
        ideal_ang = atan((self.goaly - self.posy) / (self.goalx - self.posx))

        # Log real and ideal angles
        self.get_logger().info("[ real angle, ideal angle ] : [ %s, %s ]" % (real_ang, ideal_ang))

        # Calculate error between current and target positions
        error = sqrt((self.goalx - self.posx) ** 2 + (self.goaly - self.posy) ** 2)

        # Check if the robot has reached the target position
        if self.goalx - self.posx < 0.0 and self.goaly - self.posy < 0.0:
            # Stop the robot and plot the graphs
            linear_vel = 0.0
            self.plot_graphs()
        else:
            # Set linear velocity based on error
            linear_vel = 5.0

        # Update robot's position based on real angle and linear velocity
        self.posx += cos(real_ang) * tstep * linear_vel / 10
        self.posy += sin(real_ang) * tstep * linear_vel / 10

        # Log robot's current position
        self.get_logger().info("[ x, y ] : [ %s, %s ]" % (self.posx, self.posy))

        # Log linear velocity
        self.get_logger().info("[ linear velocity ] : [ %s ]" % linear_vel)

        # Prepare joint position and wheel velocity messages
        joint_positions = Float64MultiArray()
        wheel_velocities = Float64MultiArray()

        # Set wheel velocities based on the difference between real and ideal angles
        wheel_velocities.data = [-linear_vel, -linear_vel, linear_vel]

        # Set joint positions based on the difference between real and ideal angles
