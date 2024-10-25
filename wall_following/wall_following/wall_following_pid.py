import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class WallFollowingPIDNode(Node):
    def __init__(self):
        super().__init__('wall_following_pid_node')
        
        # Parameters for wall following
        self.track_width = 0.6  # Approximate width of the track (meters)
        self.base_speed = 0.3  # Base forward speed (m/s)
        self.min_speed = 0.1  # Minimum speed when close to obstacles
        self.max_speed = 0.5  # Maximum speed in open spaces
        self.stop_distance = 0.5  # Minimum distance to stop for an obstacle in front
        
        # PID gains
        self.Kp = 2.0  # Proportional gain for centering
        self.Ki = 0.0  # Integral gain
        self.Kd = 0.5  # Derivative gain
        
        # Variables for PID control
        self.prev_error = 0.0
        self.integral_error = 0.0
        self.prev_time = self.get_clock().now()
        
        # Subscribers and publishers
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('Centering PID Node with Obstacle Avoidance has been started.')
    
    def laser_callback(self, msg):
        # Define angular limits for directions based on actual angles
        front_scan = min(self.get_scan_range(msg, 2.356, 3.927))     # Front view (~135 to 225 degrees)
        right_scan = min(self.get_scan_range(msg, 0.785, 2.356))     # Right side (~45 to 135 degrees)
        left_scan = min(self.get_scan_range(msg, -2.356, -0.785))    # Left side (~-135 to -45 degrees)
        
        # Calculate the centering error as the difference between left and right distances
        error = (left_scan - right_scan) / 2.0  # Positive if left > right, meaning move right
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        
        # PID calculations
        derivative_error = (error - self.prev_error) / dt
        self.integral_error += error * dt
        angular_correction = self.Kp * error + self.Kd * derivative_error

        # Log the error and derivative error for debugging
        self.get_logger().info(
            f"Centering Error: {error:.2f}, Integral Error: {self.integral_error:.2f}, Derivative Error: {derivative_error:.2f}"
        )

        # Adjust angular_z directly for centering response
        angular_z = max(-1.0, min(angular_correction, 1.0))  # Clamp between -1 (right) and +1 (left)

        # Speed adjustment based on proximity to obstacles in front
        if front_scan < self.stop_distance:
            self.get_logger().info("Close to front obstacle")
            linear_x = 0.0  # Stop if an obstacle is too close
        else:
            linear_x = self.base_speed * (front_scan / self.stop_distance)
            linear_x = max(self.min_speed, min(linear_x, self.max_speed))  # Clamp speed
        
        # Create and publish the Twist message
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_x
        cmd_vel.angular.z = angular_z
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Logging for debugging
        self.get_logger().info(
            f"Front scan: {front_scan:.2f}, Right scan: {right_scan:.2f}, "
            f"Left scan: {left_scan:.2f}, Centering Error: {error:.2f}, Speed: {cmd_vel.linear.x:.2f}, Angular z: {cmd_vel.angular.z:.2f}"
        )
        
        # Update previous values
        self.prev_error = error
        self.prev_time = current_time

    def get_scan_range(self, msg, start_angle, end_angle):
        """Return a list of distances within the specified angular range."""
        start_index = int((start_angle - msg.angle_min) / msg.angle_increment)
        end_index = int((end_angle - msg.angle_min) / msg.angle_increment)

        # Ensure indices are within the bounds of the scan data
        start_index = max(0, start_index)
        end_index = min(len(msg.ranges) - 1, end_index)

        return msg.ranges[start_index:end_index + 1]

def main(args=None):
    rclpy.init(args=args)
    wall_following_pid_node = WallFollowingPIDNode()
    rclpy.spin(wall_following_pid_node)
    wall_following_pid_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
