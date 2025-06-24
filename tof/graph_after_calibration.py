import numpy as np
import matplotlib.pyplot as plt

# Provided data
measured = np.array([18, 20, 22, 29, 35, 38, 45, 48, 55, 60, 65, 71, 75, 80, 84, 89, 95, 99, 111,
                     120, 130, 141, 149, 160, 171, 179, 191, 197, 211, 222, 234, 239, 254, 264, 270,
                     282, 288, 295])
true = np.array([15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 110,
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
