from machine import Pin, PWM
import time
from WaveShaker import *

# Motor setup
freq = 1000
left_motor = PWM(Pin(14), freq=freq)
left_motor_back = PWM(Pin(12), freq=freq)
right_motor = PWM(Pin(27), freq=freq)
right_motor_back = PWM(Pin(26), freq=freq)

# --- PID Tuning Constants ---
KP_LINE         = 80.0
KI_LINE         = 0.5
KD_LINE         = 500.0
INTEGRAL_MAX    = 100
BASE_SPEED      = 700

# --- Movement logic ---
robot_heading = 0

def set_speed(pwm, duty):
    pwm.duty(duty)

def Stop():
    set_speed(left_motor, 0)
    set_speed(left_motor_back, 0)
    set_speed(right_motor, 0)
    set_speed(right_motor_back, 0)

ir = IRSensorArray()

integral_line_error = 0.0
last_line_error = 0.0
last_loop_time = time.ticks_ms()

Stop()
print("[PID] Starting line-follow mode using WaveShaker.IRSensorArray")

while True:
    current_time = time.ticks_ms()
    dt = time.ticks_diff(current_time, last_loop_time) / 1000.0
    last_loop_time = current_time

    binary = ir.read_binary()
    weights = [-2, -1, 0, 1, 2]
    total_weighted_sum = 0
    black_sensors_count = 0
    for val, w in zip(binary, weights):
        if val == 0:
            total_weighted_sum += w
            black_sensors_count += 1
    err_line = (total_weighted_sum / black_sensors_count) if black_sensors_count else None
    print(f"Binary: {binary}, Err: {err_line}")

    if err_line is None:
        Stop()
        print("Line lost. Searching...")
        time.sleep(0.1)
        continue

    proportional_term = KP_LINE * err_line
    integral_line_error += err_line * dt
    integral_line_error = max(-INTEGRAL_MAX, min(INTEGRAL_MAX, integral_line_error))
    integral_term = KI_LINE * integral_line_error
    derivative_term = KD_LINE * ((err_line - last_line_error) / dt) if dt > 0 else 0.0
    last_line_error = err_line

    adjust = proportional_term + integral_term + derivative_term

    speed_r = BASE_SPEED - adjust
    speed_l = BASE_SPEED + adjust

    set_speed(left_motor, max(0, min(1023, int(speed_l))))
    set_speed(left_motor_back, 0)
    set_speed(right_motor, max(0, min(1023, int(speed_r))))
    set_speed(right_motor_back, 0)

    time.sleep(0.02)

