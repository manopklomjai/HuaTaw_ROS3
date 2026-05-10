🏎️ F1 Talent: Autonomous Racing Car (ROS 2)

Welcome to the F1 Talent project repository. This project implements a ROS 2-based autonomous racing system for a scale car using a VESC motor controller, LDS-01 LiDAR, and BMI160 IMU.  

📌 Project Overview

The system is designed to navigate a track autonomously using a "Wall Following" logic on the left side while maintaining obstacle avoidance capabilities for frontal hazards.  

🏗️ System Architecture

The software is organized into several modular nodes communicating via ROS 2 topics:
1. Control & Autonomy

    autonomous_vesc.py: The core decision-making node.  

    Logic: Implements a P-Controller for left-wall following at a target distance of 0.45m.  

Safety: If an obstacle is detected within 0.7m at the front, the car automatically executes a hard right turn to avoid a collision.  

Actuation: Publishes to /commands/motor/duty_cycle and /commands/servo/position.  

2. Sensor Drivers

    lidar_high_freq_driver.py: A high-performance driver for the LDS-01 LiDAR.  

    Operates at 230400 baud via Serial to ensure low latency.  

Publishes standard sensor_msgs/LaserScan data.  

imu_node.py: Driver for the BMI160 IMU.  

    Handles sensor calibration and bias compensation.  

Publishes raw IMU data and broadcasts the TF Transform from base_link to imu_link.  

imu_yaw.py / imu_yaw_reader.py: Specialized nodes for integrating Z-axis angular velocity to calculate the car's Yaw (heading) in degrees.  

3. Telemetry & Physical Data

    vesc_to_speed.py: Converts raw VESC electrical data (ERPM) into physical speed (m/s).  

    Parameters: Gear Ratio: 6.88, Wheel Radius: 0.033m, Pole Pairs: 2.  

⚙️ Hardware Configuration
Component	Interface	Key Specifications
Motor Controller	VESC	

Duty Cycle Control

LiDAR	LDS-01 (Serial)	

Range: 0.12m - 3.5m

IMU	BMI160 (I2C)	

16384 LSB/g sensitivity

Drive System	RWD / AWD	

6.88 Gear Reduction

🚀 Installation & Setup

    Dependencies:
    Ensure you have rclpy, pyvesc, serial, and numpy installed on your Jetson Nano/Linux environment.  

Package Build:
The package is named my_car_package. Build it using Colcon:  

Bash

colcon build --packages-select my_car_package
source install/setup.bash

Permissions:
Ensure your user has access to serial ports:
Bash

    sudo chmod 666 /dev/ttyACM0  # VESC
    sudo chmod 666 /dev/ttyUSB0  # LiDAR

🎮 How to Run
Test Motor Spin

Check VESC connectivity with a simple 5% duty cycle test:  

Bash

python3 simple_spin.py

Autonomous Mode

Launch the autonomous control node:  

Bash

ros2 run my_car_package auto_vesc_control

Monitor Telemetry

View the calculated speed in m/s:  

Bash

ros2 run my_car_package speed_calc

📐 Mathematical Logic

    Steering Control:
    The steering uses a proportional error based on the left wall distance:
      

error=target_dist(0.45m)−current_left_dist

  

steering=center(0.5)+(error×1.3)

  

Speed Calculation:
The linear speed is derived from ERPM:
  

Speed(m/s)=60(PolePairsERPM​×GearRatio1​)×(2π×WheelRadius)​
