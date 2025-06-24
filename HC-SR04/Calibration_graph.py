import numpy as np
import matplotlib.pyplot as plt

# Your data in centimeters
expected = np.array([10, 15, 20, 25, 30, 35, 40, 45, 50])
measured = np.array([10.22, 14.31, 19.22, 24.48, 29.41, 34.26, 39.2, 44.14, 48.33])

# Plotting
plt.figure(figsize=(8, 5))
plt.plot(expected, measured, 'bo-', label='Measured Distance')
plt.plot(expected, expected, 'k--', label='Ideal (Expected = Measured)')
plt.xlabel('Expected Distance (cm)')
plt.ylabel('Measured Distance (cm)')
plt.title('Ultrasonic Sensor Calibration: Measured vs. Expected')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
