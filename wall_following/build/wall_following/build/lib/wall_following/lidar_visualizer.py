import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from math import cos, sin  # Add this line to import cos and sin functions

class LidarVisualizer(Node):
    def __init__(self):
        super().__init__('lidar_visualizer')
        
        # Create a subscriber for the LiDAR data
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        
        # Create a publisher for the RViz Marker
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        
        self.get_logger().info('Lidar Visualizer Node has been started.')

    def laser_callback(self, msg):
        # Create a Marker object
        marker = Marker()
        marker.header.frame_id = "base_link"  # Assuming LiDAR frame is base_link
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lidar_points"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        
        # Marker properties
        marker.scale.x = 0.1  # Size of the points
        marker.scale.y = 0.1
        marker.color.a = 1.0  # Set the alpha to 1.0 (opaque)
        marker.color.r = 1.0  # Red points

        # Loop through the LiDAR ranges and convert to 3D points
        for i, distance in enumerate(msg.ranges):
            if distance < msg.range_max:
                angle = msg.angle_min + i * msg.angle_increment  # Calculate the angle of the point
                
                # Convert polar coordinates (range, angle) to Cartesian (x, y)
                point = Point()
                point.x = distance * cos(angle)
                point.y = distance * sin(angle)
                point.z = 0.0  # LiDAR is 2D, so z is 0
                
                # Add the point to the marker
                marker.points.append(point)

        # Publish the marker
        self.marker_pub.publish(marker)
        
        self.get_logger().info(f"Published {len(marker.points)} points to RViz.")

def main(args=None):
    rclpy.init(args=args)
    lidar_visualizer = LidarVisualizer()
    rclpy.spin(lidar_visualizer)
    lidar_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
