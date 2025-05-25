# sensors/encoder.py

import RPi.GPIO as GPIO
import time

class Encoder:
    def __init__(self, pin_a, pin_b=None):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.ticks = 0
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.pin_a, GPIO.BOTH, callback=self._tick)

    def _tick(self, channel):
        self.ticks += 1

    def get_ticks(self):
        return self.ticks, self.ticks

    def get_distance(self):
        return self.ticks * 0.01  # Calibrate later
