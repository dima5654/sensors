from machine import Pin, time_pulse_us
import time
from tof_data import get_stable_reading, interpolate
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
last_state_A = 0
last_state_B = 0

# === Encoder handlers ===
def encoderA_handler(pin):
    global encoder_count_A, last_state_A
    a = encA_A.value()
    b = encA_B.value()
    if a != last_state_A:
        encoder_count_A += 1 if a == b else -1
    last_state_A = a

def encoderB_handler(pin):
    global encoder_count_B, last_state_B
    a = encB_A.value()
    b = encB_B.value()
    if a != last_state_B:
        encoder_count_B += 1 if a == b else -1
    last_state_B = a

encA_A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=encoderA_handler)
encB_A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=encoderB_handler)

# === Motor control functions ===
def motor_A_forward(): motorA1.value(1); motorA2.value(0)
def motor_A_backward(): motorA1.value(0); motorA2.value(1)
def motor_B_forward(): motorB1.value(1); motorB2.value(0)
def motor_B_backward(): motorB1.value(0); motorB2.value(1)
def stop_motors(): motorA1.value(0); motorA2.value(0); motorB1.value(0); motorB2.value(0)

# === Median filter from Median_filter.py ===
TRIG = Pin(5, Pin.OUT)
ECHO = Pin(17, Pin.IN)
distance_buffer = []

def get_ultrasonic_filtered():
    global distance_buffer
    TRIG.off()
    time.sleep_us(2)
    TRIG.on()
    time.sleep_us(10)
    TRIG.off()
    duration = time_pulse_us(ECHO, 1, 30000)
    if duration < 0:
        return -1
    d = (duration / 2) / 29.1
    d = round(d, 2)
    distance_buffer.append(d)
    if len(distance_buffer) > 3:
        distance_buffer.pop(0)
    sorted_vals = sorted(distance_buffer)
    mid = len(sorted_vals) // 2
    return sorted_vals[mid] if sorted_vals else -1

# === IR sensor calibration from IR_sensor_5_output.py ===
sensor_pins = [15, 2, 25, 26, 27]
sensors = [Pin(pin, Pin.IN) for pin in sensor_pins]

def read_ir_sensors():
    return [s.value() for s in sensors]

def average(lst):
    return [round(sum(x[i] for x in lst) / len(lst)) for i in range(len(lst[0]))]

# Run IR calibration
print("Calibrating: place robot over BLACK line...")
time.sleep(3)
black_values = [read_ir_sensors() for _ in range(100)]
time.sleep(0.01)

print("Now place robot over WHITE surface...")
time.sleep(5)
white_values = [read_ir_sensors() for _ in range(100)]
time.sleep(0.01)

black_avg = average(black_values)
white_avg = average(white_values)
thresholds = [int((b + w) / 2) for b, w in zip(black_avg, white_avg)]
print("Suggested IR thresholds:", thresholds)

# === Start motors ===
motor_A_backward()
motor_B_forward()
start_time = time.ticks_ms()
print("Motors running for 5 seconds...")

# === Main Loop ===
while time.ticks_diff(time.ticks_ms(), start_time) < 5000:
    # ToF
    raw = get_stable_reading()
    corrected = interpolate(raw)

    # Ultrasonic
    us_dist = get_ultrasonic_filtered()

    # IMU
    ax, ay, az = get_acceleration()
    gx, gy, gz = get_gyro()

    # IR
    ir_vals = read_ir_sensors()

    print(f"ToF: {corrected}mm | US: {us_dist}cm | "
          f"Accel[g]: X={ax:.2f}, Y={ay:.2f}, Z={az:.2f} | "
          f"Gyro[°/s]: X={gx:.2f}, Y={gy:.2f}, Z={gz:.2f} | "
          f"Enc A={encoder_count_A}, B={encoder_count_B} | IR: {ir_vals}")

    time.sleep(0.2)

# === Stop motors ===
stop_motors()
print("Motors stopped.")
print(f"Final Encoder Count: A={encoder_count_A}, B={encoder_count_B}")
