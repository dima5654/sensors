from machine import Pin
import time

# Define your sensor pins
sensor_pins = [32, 33, 25, 26, 27]
sensors = [Pin(pin, Pin.IN) for pin in sensor_pins]

black_values = []
white_values = []
# Read all sensors at once
def read_sensors():
    return [s.value() for s in sensors]

# Average (More flexible if sensor's output switched to analog)
def average(lst):
    return [round(sum(x[i] for x in lst) / len(lst)) for i in range(len(lst[0]))]

# === START CALIBRATION ===

print("Calibrating: Place robot over BLACK line...")
time.sleep(3)
black_values = []
for _ in range(100):
    black_values.append(read_sensors())
    print("Sensor data:", read_sensors())
    time.sleep(0.01)

print("Now place robot over WHITE surface...")
time.sleep(5)  # Replace `input()` with delay or a button

white_values = []
for _ in range(100):
    white_values.append(read_sensors())
    print("Sensor data:", read_sensors())
    time.sleep(0.01)

# Compute thresholds
black_avg = average(black_values)
white_avg = average(white_values)
thresholds = [int((b + w)) / 2 for b, w in zip(black_avg, white_avg)]

print("Suggested thresholds:", thresholds)

