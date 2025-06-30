# File: imu_yaw_tracker.py

from machine import Pin, I2C
from time import ticks_ms, ticks_diff, sleep
from math import degrees
from mpu6500 import MPU6500  # This must be available in your project

# === Class for MPU6500 Yaw Tracking ===
class YawTracker:
    def __init__(self, scl_pin=22, sda_pin=21, freq=400000):
        self.i2c = I2C(0, scl=Pin(scl_pin), sda=Pin(sda_pin), freq=freq)
        self.imu = MPU6500(self.i2c)
        self.heading = 0.0
        self.gz_offset = self.calibrate_gyro()
        self.last_time = ticks_ms()

    def calibrate_gyro(self, samples=200):
        print("[YawTracker] Calibrating gyro... keep still")
        sum_z = 0
        for _ in range(samples):
            _, _, gz = self.imu.gyro
            sum_z += gz
            sleep(0.01)
        offset = sum_z / samples
        print("[YawTracker] Calibration done. Offset gz =", offset)
        return offset

    def update_heading(self):
        now = ticks_ms()
        dt = ticks_diff(now, self.last_time) / 1000.0  # seconds
        self.last_time = now

        _, _, gz = self.imu.gyro
        gz -= self.gz_offset

        self.heading += degrees(gz) * dt
        self.heading %= 360

        return self.heading
