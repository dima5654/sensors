from machine import Pin, I2C
from time import sleep
from mpu6500 import MPU6500  # Make sure this matches your file

# Turn on LED (optional)
# I2C config
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
imu = MPU6500(i2c)

while True:
    ax1, ay1, az1 = imu.acceleration
    ax = ax1 / 9.806
    ay = ay1 / 9.806
    az = az1 / 9.806
    
    ax_offset = 0.00157616 * ax + -0.003359203
    ay_offset = -0.001819218 * ay + -0.006026982
    az_offset = 0.007702093 * az + -0.0148289
    
    ax_with_offset = ax - ax_offset
    ay_with_offset = ay - ay_offset
    az_with_offset = az - az_offset
    
    
    print("ax:", ax,"\t","ax_new", ax_with_offset, "ay:", ay,"\t","ay_new", ay_with_offset, "az:", az,"\t","az_new", az_with_offset)
    sleep(0.2)

