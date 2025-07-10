from machine import Pin
import time

class Encoder:
    def __init__(self, pin_a, pin_b):
        self.pin_a = Pin(pin_a, Pin.IN)
        self.pin_b = Pin(pin_b, Pin.IN)
        self.count = 0
        self.pin_a.irq(trigger=Pin.IRQ_RISING, handler=self._handler)

    def _handler(self, pin):
        #if self.pin_b.value() == 0:
        self.count += 1
        #else:
            #self.count -= 1

    def reset(self):
        self.count = 0

    def get_count(self):
        return self.count
    

#encoder_A = Encoder(18, 5)  # Right motor encoder
#encoder_B = Encoder(16, 17)  # Left motor encoder
#while True:
 #       count_left = encoder_B.get_count()
  #      count_right = encoder_A.get_count()

# Reset for next cycle
        #encoder_B.reset()
        #encoder_A.reset()

# Speed correction based on difference
      #  correction = (count_right - count_left) * 5  # tuning factor


# Clip to valid PWM range
        
       # print(f"Encoders L: {count_left}, R: {count_right}, Corr: {correction}")
    
       # time.sleep(0.05)
