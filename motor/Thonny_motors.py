from machine import Pin, PWM
import time

# Set up PWM for motor A
pwmA1 = PWM(Pin(33), freq=1000)
pwmA2 = PWM(Pin(32), freq=1000)

# Set up PWM for motor B
pwmB1 = PWM(Pin(25), freq=1000)
pwmB2 = PWM(Pin(26), freq=1000)

def left_motor_forward(speed):
    pwmA1.duty(int(speed))   # speed: 0 to 1023
    pwmA2.duty(0)
    
def left_motor_backward(speed):
    pwmA1.duty(0)   # speed: 0 to 1023
    pwmA2.duty(int(speed))

def right_motor_forward(speed):
    pwmB1.duty(int(speed))
    pwmB2.duty(0)
    
def right_motor_backward(speed):
    pwmB1.duty(0)
    pwmB2.duty(int(speed))

def stop_motors():
    pwmA1.duty(0)
    pwmA2.duty(0)
    pwmB1.duty(0)
    pwmB2.duty(0)

#while True:
 #   left_motor_forward(0)
  #  right_motor_forward(570)
   # print('lol')
    #time.sleep(1)
