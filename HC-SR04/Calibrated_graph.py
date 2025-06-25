import numpy as np
import matplotlib.pyplot as plt

# Original data
expected = np.array([10, 15, 20, 25, 30, 35, 40, 45, 50])
measured = np.array([10.22, 14.31, 19.22, 24.48, 29.41, 34.26, 39.2, 44.14, 48.33])

calibrated = (measured - 0.1186) / 0.9722

# Plotting
plt.figure(figsize=(8, 5))
plt.plot(expected, measured, 'bo-', label='Measured')
plt.plot(expected, calibrated, 'go-', label='Calibrated')
plt.plot(expected, expected, 'k--', label='Ideal (Expected = Measured)')

plt.xlabel('Expected Distance (cm)')
plt.ylabel('Distance (cm)')
plt.title('Ultrasonic Sensor Calibration')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
