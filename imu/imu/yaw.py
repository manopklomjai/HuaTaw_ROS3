#Python code to plot yaw angle from BMI160

# Import necessary libraries
import sys
import os
import time
import numpy as np
import matplotlib.pyplot as plt  # Added for plotting
from scipy.spatial.transform import Rotation as R
from ahrs.filters import Madgwick

# Initialize Madgwick filter
madgwick = Madgwick()

# Initialize orientation quaternion
q = np.array([1.0, 0.0, 0.0, 0.0])

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
from DFRobot_BMI160 import *

# Initialize BMI160 sensor
bmi = DFRobot_BMI160_IIC(addr = BMI160_IIC_ADDR_SDO_H)

# Constants for gyro drift compensation
CALIBRATION_INTERVAL = 100  # Calibrate gyro every 100 iterations
gyro_bias = np.zeros(3)     # Initial gyro bias

# Function to read sensor data
def read_sensor_data():
    data = bmi.get_sensor_data()
    gyro_data = np.array([data['gyro']['x'], data['gyro']['y'], data['gyro']['z']]) * (3.14 / 180.0)  # Convert gyroscope data to radians
    accel_data = np.array([data['accel']['x'], data['accel']['y'], data['accel']['z']]) / 16384.0 * 9.81 # Convert accelerometer data to m/s^2
    return gyro_data, accel_data

if __name__ == "__main__":
    # Initialize sensor
    while bmi.begin() != BMI160_OK:
        print("Initialization 6-axis sensor failed.")
        time.sleep(1)
    print("Initialization 6-axis sensor success.")

    # Initialize plot
    plt.ion()  # Turn on interactive mode
    fig, ax = plt.subplots()
    x_data, y_data = [], []

    try:
        for i in range(CALIBRATION_INTERVAL):
            gyro_data, _ = read_sensor_data()
            gyro_bias += gyro_data
            time.sleep(0.01)  # Sampling interval for gyro calibration
        
        gyro_bias /= CALIBRATION_INTERVAL  # Compute average gyro bias

        while True:
            # Get sensor data
            gyro_data, accel_data = read_sensor_data()

            # Apply gyro drift compensation
            gyro_data -= gyro_bias

            # Apply Madgwick filter to update orientation quaternion
            q = madgwick.updateIMU(q, gyro_data, accel_data)

            # Convert quaternion to rotation matrix
            r = R.from_quat(q)

            # Get yaw (heading) from rotation matrix
            #yaw = np.degrees(r.as_euler('zyx')[0])  # Assuming zyx Euler angles order, and extracting yaw
            # Get yaw (heading) from orientation quaternion
            yaw = np.degrees(2 * np.arctan2(q[3], q[0]))

            # Print yaw
            print("Yaw:", yaw)

            # Plot yaw
            x_data.append(time.time())  # Use time as x-axis
            y_data.append(yaw)
            ax.clear()  # Clear previous plot
            ax.plot(x_data, y_data)
            ax.set_xlabel('Time')
            ax.set_ylabel('Yaw (degrees)')
            plt.pause(0.1)  # Pause to allow the plot to update

    except KeyboardInterrupt:
        print("Plotting stopped.")
