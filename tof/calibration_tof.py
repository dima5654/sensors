from machine import Pin, I2C
from vl53l0x import VL53L0X
import time

i2c = I2C(0, scl=Pin(22), sda=Pin(21))
tof = VL53L0X(i2c)

known_distances = [20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200, 210, 220, 230, 240, 250, 260, 270, 280, 290, 300]
measurements = []

for d in known_distances:
    input("Place target at {} mm and press Enter...".format(d))

    # Discard first 2 readings (flush sensor)
    print("Flushing sensor...")
    for _ in range(2):
        tof.ping()
        time.sleep(0.1)

    # Now collect average
    total = 0
    for _ in range(20):
        dist = tof.ping()
        print("  →", dist)
        total += dist
        time.sleep(0.1)

    avg = total // 20
    measurements.append(avg)
    print("Average at", d, "mm:", avg, "mm\n")

# Optional: print final results
print("Known Distances:", known_distances)
print("Sensor Measurements:", measurements)

