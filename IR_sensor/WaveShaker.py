from machine import Pin
import time

# Define your sensor pins
sensor_pins = [15, 27, 4, 19, 18]
sensors = [Pin(pin, Pin.IN) for pin in sensor_pins]


# Read all sensors at once
def read_sensors():
    return [s.value() for s in sensors]


