#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray, Twist, Pose
from std_msgs.msg import Float32
import math
import numpy as np

class PIDController:
    def __init__(self, kp, ki, kd, integral_limit=10, integral_active_zone=1, derivative_filter=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.integral_active_zone = integral_active_zone
        self.derivative_filter = derivative_filter
        self.prev_error = 0
        self.integral = 0
        self.prev_derivative = 0

    def update(self, error, dt=1.0):
        # Proportional term
        proportional = self.kp * error

        # Integral term with active zone and hard limit
        if abs(error) < self.integral_active_zone:
            self.integral += error * dt
            self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
        else:
            self.integral = 0  # Reset integral if error is outside the active zone
        integral = self.ki * self.integral

        # Derivative term with filtering
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        if abs(derivative) < self.derivative_filter:
            derivative = 0  # Apply deadband
        derivative = self.kd * derivative
        
        # Low-pass filter for derivative
        derivative = self.prev_derivative + (derivative - self.prev_derivative) * self.derivative_filter
        self.prev_derivative = derivative

        # Update previous error
        self.prev_error = error

        # PID output
        output = proportional + integral + derivative
        return output

class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # Vehicle parameters
        self.wheelbase = 1.8

        # Dynamic control parameters
        self.max_linear_velocity = 6.0  # Increased max speed
        self.min_linear_velocity = 2.0  # Reduced min speed for corners
        self.target_lookahead_time = 2.0  # seconds

        # Tunable parameters
        self.k_e = 1.0  # Cross-track error gain
        self.k_heading = 1.5  # Heading error gain
        self.max_steering_angle = math.radians(45)
        
        # Dynamic PID controllers
        self.velocity_pid = PIDController(kp=0.8, ki=0.15, kd=0.3)  # Increased velocity PID gains
        self.heading_pid = PIDController(kp=1.0, ki=0.05, kd=0.1)
        self.crosstrack_pid = PIDController(kp=0.8, ki=0.02, kd=0.1)

        # ROS subscribers and publishers
        self.pose_msg_subscriber = self.create_subscription(
            PoseStamped, '/pose_msg', self.pose_msg_callback, 10)
        self.pose_info_subscriber = self.create_subscription(
            PoseArray, '/pose_info', self.pose_info_callback, 10)
        
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.velocity_pub = self.create_publisher(Float32, '/vehicle/current_vel', 10)
        self.steering_angle_pub = self.create_publisher(Float32, '/vehicle/current_steering_angle', 10)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # State variables
        self.target_pose = None
        self.current_pose = None
        self.last_update_time = self.get_clock().now()
        self.current_linear_velocity = 3.0  # Initial velocity
        self.velocity_buffer = np.zeros(10)  # Buffer for velocity smoothing

    def pose_msg_callback(self, msg: PoseStamped):
        self.target_pose = msg.pose
        self.get_logger().info(f'Received target pose: {self.target_pose}')

    def pose_info_callback(self, msg: PoseArray):
        if msg.poses:
            self.current_pose = msg.poses[1]

    def control_loop(self):
        if not (self.target_pose and self.current_pose):
            return

        # Calculate time delta
        current_time = self.get_clock().now()
        dt = (current_time - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = current_time

        # Current vehicle state
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        current_yaw = self.get_yaw_from_pose(self.current_pose)

        # Target state
        target_x = self.target_pose.position.x
        target_y = self.target_pose.position.y

        # Calculate errors
        dx = target_x - x
        dy = target_y - y
        
        # Distance to target
        target_distance = math.sqrt(dx**2 + dy**2)

        # Dynamic velocity control
        velocity_error = dx - self.current_linear_velocity * self.target_lookahead_time
        velocity_adjustment = self.velocity_pid.update(velocity_error, dt)
        
        # Smooth the velocity using a buffer
        self.velocity_buffer[:-1] = self.velocity_buffer[1:]
        self.velocity_buffer[-1] = self.current_linear_velocity + velocity_adjustment
        self.current_linear_velocity = np.mean(self.velocity_buffer)
        self.current_linear_velocity = max(
            min(self.current_linear_velocity, self.max_linear_velocity),
            self.min_linear_velocity
        )

        # Lateral Control (Pure Pursuit only)
        lookahead_distance = self.current_linear_velocity * self.target_lookahead_time
        
        # Pure Pursuit steering calculation
        alpha = math.atan2(dy, dx)  # Angle to target
        steering_angle = math.atan2(2 * self.wheelbase * math.sin(alpha), lookahead_distance)
        
        # Limit steering angle
        steering_angle = max(
            min(steering_angle, self.max_steering_angle), 
            -self.max_steering_angle
        )

        # Calculate angular velocity
        angular_velocity = self.current_linear_velocity * math.tan(steering_angle) / self.wheelbase

        # Publish commands
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = self.current_linear_velocity
        cmd_vel_msg.angular.z = angular_velocity
        self.cmd_vel_publisher.publish(cmd_vel_msg)

        # Publish velocity and steering data
        self.publish_data(self.current_linear_velocity, steering_angle)

        # Logging
        self.get_logger().info(
            f'Velocity: {self.current_linear_velocity:.2f} m/s, '
            f'Steering: {math.degrees(steering_angle):.2f}°, '
            f'Distance to Target: {target_distance:.2f}m'
        )

    def publish_data(self, velocity, steering_angle):
        velocity_msg = Float32()
        steering_angle_msg = Float32()
        velocity_msg.data = velocity
        steering_angle_msg.data = steering_angle
        self.velocity_pub.publish(velocity_msg)
        self.steering_angle_pub.publish(steering_angle_msg)

    def get_yaw_from_pose(self, pose: Pose):
        q = pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y ** 2 + q.z ** 2)
        return math.atan2(siny_cosp, cosy_cosp)

    def angle_difference(self, target_yaw, current_yaw):
        return (target_yaw - current_yaw + math.pi) % (2 * math.pi) - math.pi

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()