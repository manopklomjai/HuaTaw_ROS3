import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ObstacleDetectionNode(Node):
    def __init__(self):
        super().__init__('obstacle_detection_node')

        # Threshold for obstacle detection (meters)
        self.threshold_distance = 0.1

        # LiDAR scan subscription
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.get_logger().info("Obstacle Detection Node started, looking for obstacles within 0.1m")

    def scan_callback(self, msg):
        # Define angular limits for directions based on actual angles
        front_angles = self.get_angle_range_indices(msg, 2.356, 3.927)    # ~ 135 to 225 degrees
        right_angles = self.get_angle_range_indices(msg, 0.785, 2.356)   # ~ 45 to 135 degrees
        back_angles = self.get_angle_range_indices(msg, -0.785, 0.785)  # ~ -45 to 45 degrees
        left_angles = self.get_angle_range_indices(msg, -2.356, -0.785)  # ~ -135 to -45 degrees

        # Detect obstacles within each direction
        front_scan = self.detect_obstacle(msg, msg.ranges, front_angles, 'front')
        right_scan = self.detect_obstacle(msg, msg.ranges, right_angles, 'right')
        back_scan = self.detect_obstacle(msg, msg.ranges, back_angles, 'back')
        left_scan = self.detect_obstacle(msg, msg.ranges, left_angles, 'left')

    def get_angle_range_indices(self, msg, start_angle, end_angle):
        """
        Calculate indices within the specified angular range.
        start_angle and end_angle should be in radians.
        """
        start_index = int((start_angle - msg.angle_min) / msg.angle_increment)
        end_index = int((end_angle - msg.angle_min) / msg.angle_increment)
        
        # Adjust to fit within the bounds of the LaserScan array
        start_index = max(0, start_index)
        end_index = min(len(msg.ranges) - 1, end_index)
        
        return list(range(start_index, end_index + 1))

    def detect_obstacle(self, msg, ranges, indices, direction):
        direction_ranges = [ranges[i] for i in indices if ranges[i] < float('inf')]  # Filter valid ranges
        detected_indices = [i for i in indices if ranges[i] < self.threshold_distance]
        
        # If an obstacle is detected, calculate angle range
        if detected_indices:
            min_angle = detected_indices[0] * msg.angle_increment + msg.angle_min
            max_angle = detected_indices[-1] * msg.angle_increment + msg.angle_min
            min_distance = min(direction_ranges) if direction_ranges else float('inf')
            self.get_logger().info(f"Obstacle detected in the {direction} between {min_angle:.2f}° and {max_angle:.2f}° at {min_distance} m")
        else:
            self.get_logger().info(f"No obstacle in the {direction}")
        
        return min(direction_ranges) if direction_ranges else float('inf')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
