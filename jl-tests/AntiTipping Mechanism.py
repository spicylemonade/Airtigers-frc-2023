import matplotlib.pyplot as plt
import numpy as np
# import time
# import busio
#import math
#import digitalios
#import adafruit_lis3dh

#ic2 = busio.I2C(board.Accelerometer_SCL, board.Accelerometer_SDA)
#_intl = digitalio.DigitalInOut(board.Accelerometer_INTERRUPT)
#lis3dh = adafruit_lis3dh.LIS3DH_I2C(i2c, address = 0x19, intl=_intl)
#lis3dh.range = adafruit_lis3dh.RANGE_8_G

#while True:

   #   ax, ay, az = lis3dh.acceleration
   #  anorm = math.sqrt(ax*ax + ay*ay + az*az)
   # axbar = ax/anorm
   # aybar = ay/anorm
   # azbar = az/anorm
    #theta = math.asin(axbar)*180.0/math.pi
    #roll = math.atan2(aybar, azbar)*180.0/math.pi
    
    
# Sample data for roll, pitch, and yaw angles
# Replace this with your actual data from the IMU
roll = np.random.randn(100)
pitch = np.random.randn(100)
yaw = np.random.randn(100)

# Compute theta, angle, and roll
theta = np.arctan2(-roll, np.sqrt(pitch**2 + yaw**2))
angle = np.arctan2(pitch, yaw)
roll_angle = roll

# Plot roll, pitch, and yaw angles
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 10))
ax1.plot(roll, label='Roll')
ax1.legend()
ax2.plot(pitch, label='Pitch')
ax2.legend()
ax3.plot(yaw, label='Yaw')
ax3.legend()

# Print the roll angle
print(f"Roll angle: {roll_angle[0]}")

plt.show()
