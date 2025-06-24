from machine import Pin, I2C
from vl53l0x import VL53L0X
import time

# Initialize I2C and ToF sensor
i2c = I2C(0, scl=Pin(22), sda=Pin(21))
tof = VL53L0X(i2c)

# Calibration table: (measured_value, actual_distance)
calibration = [
    (37, 30), (40, 35), (45, 40), (49, 45), (53, 50), (56, 55), (62, 60), (68, 65), (75, 70), (80, 75),
    (86, 80), (93, 85), (99, 90), (104, 95), (111, 100), (118, 110), (130, 120), (141, 130), (150, 140), (162, 150), (170, 160),
    (181, 170), (188, 180), (198, 190), (209, 200), (217, 210), (225, 220), (233, 230), (244, 240), (248, 250), (263, 260),
    (273, 270), (262, 280), (274, 290), (281, 300)
]

# Linear interpolation based on calibration
def interpolate(measured):
    for i in range(len(calibration) - 1):
        x0, y0 = calibration[i]
        x1, y1 = calibration[i + 1]
        if x0 <= measured <= x1:
            return int(y0 + (y1 - y0) * (measured - x0) / (x1 - x0))
    return measured  # fallback: outside range

# Get stable average from multiple readings
def get_stable_reading(samples=10, delay=0.05):
    readings = []
    for _ in range(samples):
        reading = tof.ping()
        if reading > 0:  # Filter out invalid 0s
            readings.append(reading)
        time.sleep(delay)
    if not readings:
        return 0
    return sum(readings) // len(readings)

# Main loop
while True:
    raw = get_stable_reading()
    corrected = interpolate(raw)
    print("Raw:", raw, "→ Corrected:", corrected, "mm")
    time.sleep(0.1)

