from pyb import Pin, Timer
from time import ticks_us, ticks_diff  # Use to get dt value in update()

class Controller:

    def __init__(self, kp, ki, kd, setpoint=0.0, u_min=-100, u_max=100):           # set up class arguments
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.setpoint = float(setpoint)

        self._u_min = float(u_min)
        self._u_max = float(u_max)

        self.last_error = 0.0
        self.integral_error = 0.0
        self.last_time = None 

    def set_setpoint(self, new_setpoint):
        self.setpoint = float(new_setpoint)
        self.last_error = 0.0 
        self.integral_error = 0.0  # Reset integral term when setpoint changes
        self.last_time = None

    def update(self, current_measure):   # update method
        
        current_time = ticks_us()
        if self.last_time is None:                      # if there is no last time
            self.last_time = current_time               # the current time is the last time
            return 0.0                                  # No output on first call

        dt_us = float(ticks_diff(current_time, self.last_time))
        if dt_us <= 0:
            return 0.0
        dt = dt_us*1e-6

        error = self.setpoint - float(current_measure)        # the error is the setpoint - actual position\

        p = self.kp*error
        self.integral_error += self.ki*error*dt

        d = 0.0
        if self.kd != 0.0:
            d = self.kd*(error-self.last_error)/dt

        u = p + self.integral_error + d

        if u > self._u_max:                             # creates anti-windup through clamping
            u = self._u_max
        elif u < self._u_min:
            u = self._u_min
      
        self.last_error = error                         # make the last error the current error
        self.last_time = current_time                   # make the last time the current time

        return u
    
    def reset(self, setpoint=None):                     # Clear integratior / history and optionally set a new setpoint
        if setpoint is not None:
            self.setpoint = float(setpoint)             
        self.last_error = 0.0
        self.integral_error = 0.0
        self.last_time = None
