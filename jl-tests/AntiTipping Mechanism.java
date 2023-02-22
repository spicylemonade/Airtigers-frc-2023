import matplotlib.pyplot as plt
import numpy as np

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