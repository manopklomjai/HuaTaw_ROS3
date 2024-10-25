import Jetson.GPIO as GPIO
import time
import matplotlib.pyplot as plt

# Set up GPIO mode
GPIO.setmode(GPIO.BOARD)

# Define the pin number
pin = 15  # GPIO pin 13 on Jetson Nano corresponds to physical pin 33

# Set up GPIO pin as input
GPIO.setup(pin, GPIO.IN)

# Initialize lists to store time and status data for plotting
times = []
statuses = []

# Create a figure and axis for plotting
plt.ion()  # Turn on interactive mode
fig, ax = plt.subplots()

try:
    while True:
        # Read the status of GPIO pin
        status = GPIO.input(pin)
        
        # Get current time
        current_time = time.time()
        
        # Append time and status to lists
        times.append(current_time)
        statuses.append(status)
        
        # Limit data to last 50 points for better visualization
        if len(times) > 50:
            times = times[-50:]
            statuses = statuses[-50:]
        
        # Clear axis and plot data
        ax.clear()
        ax.plot(times, statuses)
        ax.set_xlabel('Time')
        ax.set_ylabel('GPIO Pin Status')
        ax.set_title('Status of PWM Pin (GPIO {})'.format(pin))
        
        # Draw the plot
        plt.draw()
        plt.pause(0.1)  # Pause to allow plot to update
        
except KeyboardInterrupt:
    # Clean up GPIO on keyboard interrupt
    GPIO.cleanup()
    plt.close()
