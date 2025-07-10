from machine import Pin, PWM
import time
from WaveShaker import *

# Motor setup
freq = 1000
left_motor = PWM(Pin(14), freq=freq)
left_motor_back = PWM(Pin(12), freq=freq)
right_motor = PWM(Pin(27), freq=freq)
right_motor_back = PWM(Pin(26), freq=freq)

electromagnet = Pin(19, Pin.OUT)



# --- Movement logic ---

def set_speed(pwm, duty):
    pwm.duty(duty)

def forward(vel=500): 
    set_speed(left_motor, vel)
    set_speed(left_motor_back, 0)
    set_speed(right_motor, vel)
    set_speed(right_motor_back, 0)

def left(vel=500):
    set_speed(left_motor, 0)
    set_speed(left_motor_back, vel)
    set_speed(right_motor, vel)
    set_speed(right_motor_back, 0)

def right(vel=500):
    set_speed(left_motor, vel)
    set_speed(left_motor_back, 0)
    set_speed(right_motor, 0)
    set_speed(right_motor_back, vel)

def Stop():
    set_speed(left_motor, 0)
    set_speed(left_motor_back, 0)
    set_speed(right_motor, 0)
    set_speed(right_motor_back, 0)
    
while True:
    forward(700)
    electromagnet.on()  # Turns on the electromagnet
    print("forward")
    time.sleep(10)
    left(700)
    print("left")
    time.sleep(2)
    right(700)
    print("right")
    time.sleep(2)

