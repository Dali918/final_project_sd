import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import time

class RealTimePlotter(Node):
    def __init__(self):
        super().__init__('real_time_plotter')
        
        # Data storage with timestamps
        self.velocity_data = []
        self.steering_angle_data = []
        self.velocity_timestamps = []
        self.steering_timestamps = []
        
        # Create two subplots
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 8))
        
        # Velocity subplot
        self.velocity_line, = self.ax1.plot([], [], label='Velocity', color='blue', linewidth=2)
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Velocity (m/s)')
        self.ax1.set_title('Vehicle Velocity over Time')
        self.ax1.grid(True, linestyle='--', alpha=0.7)
        self.ax1.legend()
        
        # Steering angle subplot
        self.steering_line, = self.ax2.plot([], [], label='Steering Angle', color='red', linewidth=2)
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Steering Angle (degrees)')
        self.ax2.set_title('Steering Angle over Time')
        self.ax2.grid(True, linestyle='--', alpha=0.7)
        self.ax2.legend()
        
        # Start time for relative time calculation
        self.start_time = time.time()
        
        # ROS 2 subscribers
        self.create_subscription(Float32, '/vehicle/current_vel', self.velocity_callback, 10)
        self.create_subscription(Float32, '/vehicle/steering_angle', self.steering_angle_callback, 10)
        
        # Adjust layout to prevent overlap
        plt.tight_layout()

    def velocity_callback(self, msg):
        # Record velocity with relative timestamp
        current_time = time.time() - self.start_time
        self.velocity_data.append(msg.data)
        self.velocity_timestamps.append(current_time)
        self.update_velocity_plot()

    def steering_angle_callback(self, msg):
        # Record steering angle with relative timestamp
        current_time = time.time() - self.start_time
        self.steering_angle_data.append(msg.data)
        self.steering_timestamps.append(current_time)
        self.update_steering_plot()

    def update_velocity_plot(self):
        # Update velocity plot with timestamps
        self.velocity_line.set_data(self.velocity_timestamps, self.velocity_data)
        self.ax1.relim()
        self.ax1.autoscale_view()
        self.update_plot()

    def update_steering_plot(self):
        # Update steering angle plot with timestamps
        self.steering_line.set_data(self.steering_timestamps, self.steering_angle_data)
        self.ax2.relim()
        self.ax2.autoscale_view()
        self.update_plot()

    def update_plot(self):
        # Pause to allow plot update
        plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)
    plotter = RealTimePlotter()
    try:
        rclpy.spin(plotter)
    except KeyboardInterrupt:
        # Save the final plot before closing
        plt.savefig('vehicle_data_plot.png')
        plt.close()
    finally:
        plotter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()