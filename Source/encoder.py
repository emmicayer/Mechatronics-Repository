## @file encoder.py
#  @brief Quadrature encoder interface using a timer in encoder mode to track
#  wheel position and velocity.
#
#  This module defines the Encoder class that configures a hardware timer in
#  quadrature (ENC_A/ENC_B) mode and provides:
#    • Continuous accumulation of encoder position (with overflow handling)
#    • Measurement of incremental counts and time between updates
#    • Methods to return current position and velocity
#    • A `zero()` function to reset position and internal counters

from pyb import Pin, Timer
from time import ticks_us, ticks_diff  # Use to get dt value in update()


class Encoder:

    def __init__(self, tim: Timer, chA_pin, chB_pin):
        
        # Initialize pins
        self.chA_pin = Pin(chA_pin, mode=Pin.AF_PP)
        self.chB_pin = Pin(chB_pin, mode=Pin.AF_PP)

        # Initialize timer 
        self.tim = tim
        self.tim.init(prescaler=0, period=65535)
        self.ch1 = tim.channel(1, pin= chA_pin, mode=Timer.ENC_A)
        self.ch2 = tim.channel(2, pin= chB_pin, mode=Timer.ENC_B)

        # Tracking variables
        self.position = 0  # Total accumulated position of the encoder
        self.prev_count = self.tim.counter()  # Counter value from the most recent update
        self.delta = 0  # Change in count between last two updates
        self.dt = 0  # Amount of time between last two updates
        self.prev_time = ticks_us()
        self.AR = 65535

    def update(self):

        # Get update count and time and calculate change
        self.new_time = ticks_us()
        self.new_count = self.tim.counter()
        self.dt = ticks_diff(self.new_time, self.prev_time)
        self.prev_time = self.new_time
        self.delta = self.new_count - self.prev_count
        self.prev_count = self.new_count

        # Check for underflow/overflow
        if self.delta < -(self.AR+1)/2:
            self.delta += (self.AR+1)
        elif self.delta > (self.AR+1)/2:
            self.delta += -(self.AR+1)

        # Update position
        self.position += self.delta
    
    def get_position(self):     # Returns the most recently updated value of position as determined within the update() method
        return -self.position

    def get_velocity(self):     # Returns a measure of velocity using the the most recently updated value of delta as determined within the update() method
        if self.dt > 0:
            return -self.delta / (self.dt)
        else:
            return 0

    def zero(self):             # Sets the present encoder position to zero and causes future updates to measure with respect to the new zero position
        # self.tim.counter()
        # self.position = 0
        # self.prev_count = 0
        self.tim.counter(0)     # <-- ACTUAL reset
        self.position = 0
        self.prev_count = 0
        self.delta = 0
        self.prev_time = ticks_us()