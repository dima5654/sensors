from machine import Pin, PWM

# Set up PWM for motor A
pwmA1 = PWM(Pin(18), freq=1000)
pwmA2 = PWM(Pin(19), freq=1000)

# Set up PWM for motor B
pwmB1 = PWM(Pin(12), freq=1000)
pwmB2 = PWM(Pin(13), freq=1000)

def motor_A_forward(speed):
    pwmA1.duty(int(speed))   # speed: 0 to 1023
    pwmA2.duty(0)

def motor_B_forward(speed):
    pwmB1.duty(int(speed))
    pwmB2.duty(0)

def stop_motors():
    pwmA1.duty(0)
    pwmA2.duty(0)
    pwmB1.duty(0)
    pwmB2.duty(0)
