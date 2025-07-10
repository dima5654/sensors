from machine import Pin
from time import sleep

# Define the pin connected to the electromagnet (e.g., GPIO 5)
electromagnet = Pin(19, Pin.OUT)

print("Testing electromagnet...")

while True:
    #print("ON")
    #electromagnet.on()  # Turns on the electromagnet
    #sleep(20)             # Wait for 2 seconds
    
    print("OFF")
    electromagnet.off()  # Turns it off
    sleep(2)

