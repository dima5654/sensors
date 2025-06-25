from machine import Pin, I2C
from mpu6500 import MPU6500

i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
imu = MPU6500(i2c)

gx_offset = 0.01063169
gy_offset = -0.01223756
gz_offset = -0.00640248

def get_gyro():
    gx, gy, gz = imu.gyro
    return gx - gx_offset, gy - gy_offset, gz - gz_offset

