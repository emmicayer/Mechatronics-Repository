from pyb import Pin, Timer


class Motor:
    def __init__(self, PWM_pin, DIR_pin, nSLP_pin, TIM: Timer, CHAN):                # Initializes a motor object
       
        self.PWM_pin = Pin(PWM_pin, mode = Pin.OUT_PP)
        self.DIR_pin = Pin(DIR_pin, mode = Pin.OUT_PP)
        self.nSLP_pin = Pin(nSLP_pin, mode=Pin.OUT_PP, value=0)
        self.TIM = TIM
        self.CHAN = CHAN
        self.PWM_chan = TIM.channel(CHAN, pin= PWM_pin, mode=Timer.PWM, pulse_width_percent=0)

    def enable(self):                                           # Enables the motor driver by taking it out of sleep mode into brake mode
        self.nSLP_pin.high()

    def disable(self):                                          # Disables the motor driver by taking it into sleep mode
        self.nSLP_pin.low()

    def set_effort(self, effort: float):                        # Sets the present effort requested from the motor based on an input value between -100 and 100
        if effort > 100: effort = 100                           # Clamp to safe range just in case
        if effort < -100: effort = -100        
        if (effort > 0):
            self.DIR_pin.low()
            self.PWM_chan.pulse_width_percent(effort) 
        else: 
            self.DIR_pin.high() 
            self.PWM_chan.pulse_width_percent(-effort) 
