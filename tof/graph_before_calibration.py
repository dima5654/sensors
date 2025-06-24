import numpy as np
import matplotlib.pyplot as plt

# Provided data
measured = np.array([37, 40, 45, 49, 53, 56, 62, 68, 75, 80, 86, 93, 99, 104, 111, 118,
                     130, 141, 150, 162, 170, 181, 188, 198, 209, 217, 225, 233, 244, 248, 263, 273,
                     262, 274, 281])
true = np.array([30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 110,
                     120, 130, 140, 150, 160, 170, 180, 190, 200, 210, 220, 230, 240, 250, 260, 270,
                     280, 290, 300])

# Plotting
plt.figure(figsize=(8, 5))
plt.plot(true, measured, 'bo-', label='Measured')
plt.plot(true, true, 'k--', label='Ideal (True = Measured)')
plt.xlabel('True Distance (mm)')
plt.ylabel('Measured Distance (mm)')
plt.title('Measured vs. True Distance (VL53L0X)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
