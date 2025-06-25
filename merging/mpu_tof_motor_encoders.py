from machine import Pin
import time
from calibrated import get_stable_reading, interpolate
from gyro_data import get_gyro
from acceleration_data import get_acceleration

# === Motor control pins ===
motorA1 = Pin(18, Pin.OUT)
motorA2 = Pin(19, Pin.OUT)
motorB1 = Pin(12, Pin.OUT)
motorB2 = Pin(13, Pin.OUT)

# === Encoder pins ===
encA_A = Pin(34, Pin.IN)
encA_B = Pin(35, Pin.IN)
encB_A = Pin(32, Pin.IN)
encB_B = Pin(33, Pin.IN)

# === Encoder counters ===
encoder_count_A = 0
encoder_count_B = 0


last_state_A = 0  # place this globally
last_state_B = 0

# Interrupts
def encoderA_handler(pin):
    global encoder_count_A, last_state_A
    a = encA_A.value()
    b = encA_B.value()
    if a != last_state_A:
        if a == b:
            encoder_count_A += 1
        else:
            encoder_count_A -= 1
    last_state_A = a

def encoderB_handler(pin):
    global encoder_count_B, last_state_B
    a = encB_A.value()
    b = encB_B.value()
    if a != last_state_B:
        if a == b:
            encoder_count_B += 1
        else:
            encoder_count_B -= 1
    last_state_B = a

# Attach interrupts on both edges
encA_A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=encoderA_handler)
encB_A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=encoderB_handler)

# === Motor control functions ===
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

# === Start motors ===
motor_A_backward()
motor_B_forward()

start_time = time.ticks_ms()
print("Motors running forward for 5 seconds...")

# === Main loop with ToF, IMU, and Encoder output ===
while time.ticks_diff(time.ticks_ms(), start_time) < 5000:
    # ToF
    raw = get_stable_reading()
    corrected = interpolate(raw)

    # IMU
    ax, ay, az = get_acceleration()
    gx, gy, gz = get_gyro()

    # Output all readings
    print(f"ToF: {corrected}mm (raw: {raw}) | "
          f"Accel [g]: X={ax:.3f}, Y={ay:.3f}, Z={az:.3f} | "
          f"Gyro [°/s]: X={gx:.3f}, Y={gy:.3f}, Z={gz:.3f} | "
          f"Encoders: A={encoder_count_A}, B={encoder_count_B}")

    time.sleep(0.2)

# === Stop motors after 5 seconds ===
stop_motors()
print("Motors stopped.")
print(f"Final Encoder Count: A={encoder_count_A}, B={encoder_count_B}")

