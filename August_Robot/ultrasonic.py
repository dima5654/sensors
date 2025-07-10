from machine import Pin, time_pulse_us
import time

# Setup pins
TRIG = Pin(15, Pin.OUT)
ECHO = Pin(23, Pin.IN)

# Buffer for median filter
distance_buffer = []

def get_distance():
    TRIG.off()
    time.sleep_us(2)
    TRIG.on()
    time.sleep_us(10)
    TRIG.off()
    duration = time_pulse_us(ECHO, 1, 30000)  # Timeout at 30ms (≈500 cm)
    if duration < 0:
        return -1  # No echo
    distance_cm = (duration / 2) / 29.1
    return round(distance_cm, 2)

def get_filtered_distance():
    global distance_buffer
    d = get_distance()
    if d != -1:
        distance_buffer.append(d)
        if len(distance_buffer) > 3:
            distance_buffer.pop(0)  # Keep last 3
    if len(distance_buffer) == 0:
        return -1
    # Median calculation
    sorted_vals = sorted(distance_buffer)
    mid = len(sorted_vals) // 2
    return sorted_vals[mid]

#while True:
 #   filtered = get_filtered_distance()
  #  if filtered == -1:
   #     print("No echo received")
    #else:
     #   print("Distance (filtered):", filtered, "cm")
    #time.sleep(0.2)
