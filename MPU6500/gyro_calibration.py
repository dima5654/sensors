# Shows Pi is on by turning on LED when plugged in
from machine import Pin, I2C
from time import sleep
from mpu6500 import MPU6500  # Make sure your imu.py has MPU6500 defined
import time


# I2C configuration
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)  # Adjust SDA/SCL pins as needed
imu = MPU6500(i2c)
t0 = time.time()

settling_time = 4

print('Settling MPU for %d sconds' % settling_time)
time.sleep(4)
print('MPU is done settling')
# Loop to read and print accelerometer values
def get_gyro():
    gx, gy, gz = imu.gyro
    return gx, gy, gz


def gyro_calibration(calibration_time=20):
    print('--' * 25)
    print('Beginning of calibration, do not move the sensor')
    
    offsets = [0, 0, 0]
    
    num_of_points = 0
    
    end_loop_time = time.time() + calibration_time
    
    while end_loop_time > time.time():
        num_of_points += 1
        (gx, gy, gz) = get_gyro()
        offsets[0] += gx
        offsets[1] += gy
        offsets[2] += gz
        
        if num_of_points % 100 == 0:
            print('Still calibration... %d points so far' % num_of_points)
    print('Calibration complete %d total points' % num_of_points)
    offsets = [i/num_of_points for i in offsets]
    return offsets

print(gyro_calibration())

