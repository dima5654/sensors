from machine import ADC, Pin
import time

class IRSensorArray:
    def __init__(self, thresholds=None):
        # Initialize the ADC pins
        self.sensors = [
            ADC(Pin(34)),  # Sensor 1 (leftmost)
            ADC(Pin(35)),  # Sensor 2
            ADC(Pin(25)),  # Sensor 3 (center)
            ADC(Pin(32)),  # Sensor 4
            ADC(Pin(33))   # Sensor 5 (rightmost)
        ]

        # Default thresholds or use provided ones
        self.thresholds = thresholds if thresholds else [1000, 1300, 1900, 1250, 1900]

        # Configure ADC resolution and attenuation
        for s in self.sensors:
            s.width(ADC.WIDTH_12BIT)
            s.atten(ADC.ATTN_11DB)

    def read(self):
        """Return raw analog readings from all sensors"""
        return [s.read() for s in self.sensors]

    def read_binary(self):
        """Return 0 or 1 from each sensor based on its individual threshold"""
        raw_values = self.read()
        return [1 if raw_values[i] > self.thresholds[i] else 0 for i in range(5)]
#while True:
      #ir = IRSensorArray()
    
      #sensors = ir.read_binary()
    
      #left_val, left_mid_val, mid_val, right_mid_val, right_val = sensors[0], sensors[1], sensors[2], sensors[3], sensors[4]  # '0' means black line detected
      #print(left_val, left_mid_val, mid_val, right_mid_val, right_val)
      #time.sleep(0.5)

