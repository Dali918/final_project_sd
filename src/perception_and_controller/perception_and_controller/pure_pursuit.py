import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped, PoseArray
from nav_msgs.msg import Path
from messages.msg import VehicleState # Import the custom message type for vehicle state
import math

class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        self.logger = self.get_logger()
        self.waypoints = []

        """
        param that works: 
        self.wheelbase = 0.8
        self.k = 1.25  
        self.linear_velocity = 4.0
        self.lookahead_distance =  2.0 * self.linear_velocity
        
        """

        # Vehicle Parameters
        self.wheelbase = 0.8

        # Tunable parameters
        self.k = 0.7 
        self.linear_velocity = 5.5
        self.lookahead_distance =  1.8 * self.linear_velocity  

        # Subscribe to /gazebo/vehicle_state topic
        self.subscription = self.create_subscription(
            PoseArray,
            '/pose_info',
            self.pose_callback,
            10)

        self.path_subscriber = self.create_subscription(
            PoseStamped,
            '/pose_msg',
            self.path_callback,
            10)

        # Publisher for cmd_vel topic
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.logger.info("Pure Pursuit Controller node started")

    def path_callback(self, target_msg: PoseStamped):
        # Update the waypoint when /pose_msg is received
        self.target_pose = target_msg.pose

    def pose_callback(self, msg: PoseArray):
        if not self.waypoints:
            self.logger.warn("No target waypoint received.")
            return

        current_pose = msg.poses[1]
        current_x = current_pose.x
        current_y = current_pose.y
        yaw = self.get_yaw_from_pose(current_pose)

        # Use the single target waypoint from the /pose_msg topic
        target_waypoint = (self.target_pose.position.x, self.target_pose.position.y)

        # Calculate and publish cmd_vel message
        steering_angle = self.generate_control_output(current_x, current_y, yaw, target_waypoint)
        # Publish cmd_vel message
        twist = Twist()
        twist.linear.x = self.linear_velocity  # constant linear velocity
        twist.angular.z = self.steering_angle_to_angular_velocity(steering_angle)
        self.cmd_vel_publisher.publish(twist)

    def get_yaw_from_pose(self, pose: Pose):
        q = pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y ** 2 + q.z ** 2)
        return math.atan2(siny_cosp, cosy_cosp)

    def distance(self, point1, point2):
        return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    def steering_angle_to_angular_velocity(self, steering_angle):
        angular_velocity = math.tan(steering_angle) * self.linear_velocity / self.wheelbase
        return angular_velocity

    def generate_control_output(self, current_x, current_y, yaw, target_waypoint):
        """
        Calculates the steering angle required for path following (Use Pure Pursuit algorithm).

        Args:
            current_x (float): Current x-coordinate of the vehicle.
            current_y (float): Current y-coordinate of the vehicle.
            yaw (float): Current yaw angle (orientation) of the vehicle.

        Returns:
            float: Steering angle in radians.

        This function is called by pose_callback() to determine the necessary
        steering angle based on the vehicle's current position and orientation,
        as well as the recent waypoints automatically updated and stored in
        self.waypoints. The waypoints represent the path that the vehicle
        should follow, and the controller uses them along with self.wheelbase
        to calculate the optimal steering angle to stay on track.
        """

        if target_waypoint is None:
            self.logger.warn("No target waypoint received.")
            return 
        
        target_x, target_y = target_waypoint
        
        # Calculate the steering angle using Pure Pursuit
        alpha = math.atan2(target_y - current_y, target_x - current_x) - yaw
        alpha = (alpha + math.pi) % (2 * math.pi) - math.pi  # Normalize angle
        
        # Calculate the steering angle
        steering_angle = math.atan2(2 * self.wheelbase * math.sin(alpha), self.lookahead_distance)
        
        # Apply proportional control
        steering_angle = self.k * steering_angle
        
        # Limit the steering angle to a reasonable range (e.g., +/- 45 degrees)
        max_steering_angle = math.pi / 4  # 45 degrees
        steering_angle = max(-max_steering_angle, min(max_steering_angle, steering_angle))
        
        return steering_angle

def main(args=None):
    rclpy.init(args=args)
    controller = PurePursuitController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()