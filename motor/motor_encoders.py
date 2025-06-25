from machine import Pin
import time

# Motor A control pins
motorA1 = Pin(18, Pin.OUT)
motorA2 = Pin(19, Pin.OUT)

# Motor B control pins
motorB1 = Pin(12, Pin.OUT)
motorB2 = Pin(13, Pin.OUT)

# Encoder pins
encA_A = Pin(34, Pin.IN)
encA_B = Pin(35, Pin.IN)
encB_A = Pin(32, Pin.IN)
encB_B = Pin(33, Pin.IN)

# Encoder counters
encoder_count_A = 0
encoder_count_B = 0

# Interrupts
def encoderA_handler(pin):
    global encoder_count_A
    if encA_B.value() == 0:
        encoder_count_A += 1
    else:
        encoder_count_A -= 1

def encoderB_handler(pin):
    global encoder_count_B
    if encB_B.value() == 0:
        encoder_count_B += 1
    else:
        encoder_count_B -= 1

# Attach interrupts
encA_A.irq(trigger=Pin.IRQ_RISING, handler=encoderA_handler)
encB_A.irq(trigger=Pin.IRQ_RISING, handler=encoderB_handler)

# Motor control functions
def motor_A_forward():
    motorA1.value(1)
    motorA2.value(0)

def motor_A_backward():
    motorA1.value(0)
    motorA2.value(1)

def motor_B_forward():
    motorB1.value(1)
    motorB2.value(0)

def motor_B_backward():
    motorB1.value(0)
    motorB2.value(1)

def stop_motors():
    motorA1.value(0)
    motorA2.value(0)
    motorB1.value(0)
    motorB2.value(0)

# Main loop
motor_A_forward()
motor_B_forward()

print("Motors running forward for 5 seconds...")
time.sleep(5)

stop_motors()

print("Motor A Encoder Count:", encoder_count_A)
print("Motor B Encoder Count:", encoder_count_B)

