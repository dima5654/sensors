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
def get_acceleration():
    ax1, ay1, az1 = imu.acceleration
    ax = ax1 / 9.806
    ay = ay1 / 9.806
    az = az1 / 9.806
    return ax, ay, az


def linear_calibration(calibration_time=5, axis =0):
    num_of_points = 0
    x_sum = 0
    y_sum = 0
    x_squared_sum = 0
    x_times_y_sum = 0
    print('--' * 50)
    print('Orient the axis upwards agaist gravity - Click Enter when ready')
    
    x = input()
    end_loop_time = time.time() + calibration_time
    print('beginning to calibrate part 1 (acceleration = 1g) for %d seconds' % calibration_time)
    
    while end_loop_time > time.time():
        
        num_of_points +=1
        offset = get_acceleration()[axis] - 1
        x_sum += 1
        y_sum += offset
        x_squared_sum += 1
        x_times_y_sum += 1 * offset
        
        if num_of_points % 100 == 0:
            print('Still calibrating... %d points so far' % num_of_points)
        
    print('--' * 50)
    print('Orient the axis downwards agaist gravity - Click Enter when ready')
    
    x = input()
    end_loop_time = time.time() + calibration_time
    print('beginning to calibrate part 2 (acceleration = -1g) for %d seconds' % calibration_time)
    
    while end_loop_time > time.time():
        
        num_of_points +=1
        offset = get_acceleration()[axis] + 1
        x_sum += (-1 * 1)
        y_sum += offset
        x_squared_sum += (-1 * 1) * (-1 * 1)
        x_times_y_sum += (-1 * 1) * offset
        
        if num_of_points % 100 == 0:
            print('Still calibrating... %d points so far' % num_of_points)
    print('--' * 50)
    print('Orient the axis perpendicular agaist gravity - Click Enter when ready')
    
    x = input()
    end_loop_time = time.time() + calibration_time
    print('beginning to calibrate part 3 (acceleration = 0g) for %d seconds' % calibration_time)
    
    while end_loop_time > time.time():
        
        num_of_points +=1
        offset = get_acceleration()[axis] + 0
        x_sum += 0
        y_sum += offset
        x_squared_sum += (0) * (0)
        x_times_y_sum += (0) * offset
        
        if num_of_points % 100 == 0:
            print('Still calibrating... %d points so far' % num_of_points)
    m = (num_of_points * x_times_y_sum - (x_sum * y_sum)) / ((num_of_points * x_squared_sum) - (x_sum)**2)
    b = (y_sum - (m * x_sum)) / num_of_points
    
    return m, b

print(linear_calibration())
        
