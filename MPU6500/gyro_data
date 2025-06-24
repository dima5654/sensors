from machine import Pin, I2C
from time import sleep
from mpu6500 import MPU6500  # Make sure this matches your file

# Turn on LED (optional)
# I2C config
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
imu = MPU6500(i2c)

# Your calculated offsets
gx_offset = 0.01063169
gy_offset = -0.01223756
gz_offset = -0.00640248

while True:
    gx, gy, gz = imu.gyro

    # Apply offsets
#     gx -= gx_offset
#     gy -= gy_offset
#     gz -= gz_offset

    print("gx:", gx, "\t", "gy:", gy, "\t", "gz:", gz)
    sleep(0.2)

