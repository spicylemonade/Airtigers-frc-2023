import time
import matplotlib.pyplot as plt
from robotpy_ext.common_drivers.navx import AHRS

# Connect to NavX over USB
navx = AHRS.create_spi()

# Set up plot
fig, axs = plt.subplots(3, sharex=True)
fig.suptitle('NavX Gyroscope Data')
axs[0].set_ylabel('Roll (deg)')
axs[1].set_ylabel('Pitch (deg)')
axs[2].set_ylabel('Yaw (deg)')

# Collect and plot data
roll_data = []
pitch_data = []
yaw_data = []
while True:
    roll = navx.get_roll()
    pitch = navx.get_pitch()
    yaw = navx.get_yaw()
    print(f'Roll: {roll:.2f} deg, Pitch: {pitch:.2f} deg, Yaw: {yaw:.2f} deg')
    roll_data.append(roll)
    pitch_data.append(pitch)
    yaw_data.append(yaw)
    axs[0].plot(roll_data, 'r-')
    axs[1].plot(pitch_data, 'g-')
    axs[2].plot(yaw_data, 'b-')
    plt.pause(0.05)  # Pause for a short time to allow plot to update
