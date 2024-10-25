# Import ROS2 Python libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import Imu  # Import the standard IMU message type
from std_msgs.msg import Float32  # Import the Float32 message type for yaw angle

# Import necessary libraries
import sys
import os
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from ahrs.filters import Madgwick

# Initialize Madgwick filter
madgwick = Madgwick()

# Initialize orientation quaternion
q = np.array([1.0, 0.0, 0.0, 0.0])

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
from DFRobot_BMI160 import *

# Initialize BMI160 sensor
bmi = DFRobot_BMI160_IIC(addr=BMI160_IIC_ADDR_SDO_H)

# Constants for gyro drift compensation
CALIBRATION_INTERVAL = 100  # Calibrate gyro every 100 iterations
gyro_bias = np.zeros(3)     # Initial gyro bias
gyro_calibration_counter = 0

# Function to read sensor data
def read_sensor_data():
    data = bmi.get_sensor_data()
    gyro_data = np.array([data['gyro']['x'], data['gyro']['y'], data['gyro']['z']]) * (3.14 / 180.0)  # Convert gyroscope data to radians
    accel_data = np.array([data['accel']['x'], data['accel']['y'], data['accel']['z']]) / 16384.0 * 9.81 # Convert accelerometer data to m/s^2
    return gyro_data, accel_data

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_imu = self.create_publisher(Imu, 'imu', 10)  # Publisher for IMU data
        self.publisher_yaw = self.create_publisher(Float32, 'yaw', 10)   # Publisher for yaw angle
        self.timer_ = self.create_timer(0.1, self.publish_imu_data)
        global gyro_bias

    def publish_imu_data(self):
        global gyro_bias
        global gyro_calibration_counter
        gyro_calibration_counter += 1

        # Get sensor data
        gyro_data, accel_data = read_sensor_data()

        # Apply gyro drift compensation
        gyro_data -= gyro_bias

        # Calibrate gyro every 100 iterations
        if gyro_calibration_counter == CALIBRATION_INTERVAL:
            gyro_calibration_counter = 0
            gyro_bias = np.zeros(3)  # Reset gyro bias

        # Apply Madgwick filter to update orientation quaternion
        global q
        q = madgwick.updateIMU(q, gyro_data, accel_data)

        # Convert quaternion to rotation matrix
        global r
        r = R.from_quat(q)

        # Get yaw (heading) from orientation quaternion
        yaw = np.degrees(2 * np.arctan2(q[3], q[0]))

        # Publish IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        imu_msg.angular_velocity = Vector3(x=gyro_data[0], y=gyro_data[1], z=gyro_data[2])
        imu_msg.linear_acceleration = Vector3(x=accel_data[0], y=accel_data[1], z=accel_data[2])

        # Set covariance values
        imu_msg.orientation_covariance = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]  
        imu_msg.angular_velocity_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]  
        imu_msg.linear_acceleration_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]  

        # Publish IMU message
        self.publisher_imu.publish(imu_msg)

        # Publish yaw angle separately
        yaw_msg = Float32()
        yaw_msg.data = yaw
        self.publisher_yaw.publish(yaw_msg)

def main(args=None):
    rclpy.init(args=args)

    imu_publisher = IMUPublisher()

    rclpy.spin(imu_publisher)

    imu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
