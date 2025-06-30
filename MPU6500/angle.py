from machine import Pin, I2C
from time import ticks_ms, ticks_diff, sleep
from math import degrees
from mpu6500 import MPU6500

# I2C setup
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
imu = MPU6500(i2c)

# Gyro calibration
def calibrate_gyro(samples=200):
    print("Calibrating gyro... keep still")
    sum_z = 0
    for _ in range(samples):
        _, _, gz = imu.gyro
        sum_z += gz
        sleep(0.01)
    return sum_z / samples

gz_offset = calibrate_gyro()

# Heading initialization
heading = 0.0
last_time = ticks_ms()

while True:
    # Time step calculation
    now = ticks_ms()
    dt = ticks_diff(now, last_time) / 1000.0  # seconds
    last_time = now

    # Gyroscope reading and integration
    _, _, gz = imu.gyro
    gz -= gz_offset
    heading += degrees(gz) * dt
    heading %= 360

    print("Heading (Yaw): {:.2f}°".format(heading))

