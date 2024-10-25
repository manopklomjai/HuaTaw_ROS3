import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class WallFollowingPIDNode(Node):
    def __init__(self):
        super().__init__('wall_following_pid_node')
        
        # Parameters for wall following
        self.wall_distance = 0.5  # Desired distance from the wall (meters)
        self.speed = 0.3  # Base forward speed (m/s)
        self.min_speed = 0.1  # Minimum speed when close to obstacles
        self.max_speed = 0.5  # Maximum speed in open spaces
        self.stop_distance = 0.5  # Minimum distance to stop for an obstacle in front
        self.turn_threshold = 1.5  # Distance to start turning to avoid obstacle
        self.side_threshold = 0.3  # Threshold to detect if the robot is close to the left or right side
        
        # PID gains
        self.Kp = 0.5  # Proportional gain
        self.Ki = 0.1  # Integral gain
        self.Kd = 0.05  # Derivative gain
        
        # Variables for PID control
        self.prev_error = 0.0
        self.integral_error = 0.0
        self.prev_time = self.get_clock().now()
        
        # Subscribers and publishers
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('Wall Following PID Node with Obstacle Avoidance has been started.')
    
    def laser_callback(self, msg):
        # Get front, front-left, front-right, and right-side distances from LiDAR data
        # Assuming a 360-degree LiDAR
        front_scan = min(min(msg.ranges[0:30] + msg.ranges[-30:]), 10.0)  # Front view (close to 0 and 360 degrees)
        front_left_scan = min(min(msg.ranges[30:50]), 10.0)  # Front-left view (around 10-30 degrees)
        front_right_scan = min(min(msg.ranges[-50:-30]), 10.0)  # Front-right view (around 330-350 degrees)
        right_scan = min(min(msg.ranges[60:120]), 10.0)  # Right view (around 90 degrees)
        left_scan = min(min(msg.ranges[230:300]), 10.0)  # Left view (around 270 degrees)
        
        # Wall following PID control for staying near the wall on the right side
        error = self.wall_distance - right_scan
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        
        # PID calculations
        derivative_error = (error - self.prev_error) / dt
        self.integral_error += error * dt
        angular_z = self.Kp * error + self.Ki * self.integral_error + self.Kd * derivative_error
        
        # Obstacle avoidance: Adjust angular velocity based on obstacle location
        if front_scan < self.turn_threshold:
            if front_left_scan < front_right_scan:
                angular_z -= 1.0  # Turn left to avoid obstacle on the left
            else:
                angular_z += 1.0  # Turn right to avoid obstacle on the right

        # Speed adjustment based on proximity to obstacles in front
        if front_scan < self.stop_distance:
            linear_x = 0.0  # Stop if an obstacle is too close
        else:
            linear_x = self.speed * (front_scan / self.stop_distance)
            linear_x = max(self.min_speed, min(linear_x, self.max_speed))  # Clamp speed
        
        # Create and publish the Twist message
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_x
        cmd_vel.angular.z = angular_z
        
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Print message if close to left or right
        if right_scan < self.side_threshold:
            self.get_logger().info("Close to right side")
            angular_z += 1.0  # Turn left if too close to the right
        
        if left_scan < self.side_threshold:
            self.get_logger().info("Close to left side")
            angular_z -= 1.0  # Turn right if too close to the left
        
        # Logging for debugging
        self.get_logger().info(f"Front scan: {front_scan:.2f}, Front-left scan: {front_left_scan:.2f}, Front-right scan: {front_right_scan:.2f}, Error: {error:.2f}, Speed: {cmd_vel.linear.x:.2f}, Angular z: {cmd_vel.angular.z:.2f}")
        
        # Update previous values
        self.prev_error = error
        self.prev_time = current_time

def main(args=None):
    rclpy.init(args=args)
    wall_following_pid_node = WallFollowingPIDNode()
    rclpy.spin(wall_following_pid_node)
    wall_following_pid_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
