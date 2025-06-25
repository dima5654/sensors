from machine import Pin, I2C
from mpu6500 import MPU6500

i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
imu = MPU6500(i2c)

def get_acceleration():
    ax1, ay1, az1 = imu.acceleration
    ax = ax1 / 9.806
    ay = ay1 / 9.806
    az = az1 / 9.806

    ax_offset = 0.00157616 * ax + -0.003359203
    ay_offset = -0.001819218 * ay + -0.006026982
    az_offset = 0.007702093 * az + -0.0148289

    return ax - ax_offset, ay - ay_offset, az - az_offset

