## @file line_sensor.py
#  @brief Reflectance line sensor array driver with calibration, filtering, and
#  line position estimation.
#
#  This module implements the LineSensor class that manages an array of ADC-
#  based reflective sensors. It:
#    • Configures multiple ADC channels for the provided pins
#    • Supports oversampling and exponential moving average (EMA) filtering
#    • Calibrates sensor responses for white and black surfaces
#    • Returns normalized readings (0–1000) for each sensor
#    • Computes the line centroid position in millimeters and overall line
#      “strength” for line-following control
#

from pyb import Pin, ADC, Timer

class LineSensor:
        def __init__(self, adc_pins, pitch=4.0, oversample=4, ema=0.3):                # pitch is in mm
            assert len(adc_pins) >= 1               # check if the length of the number of adc pins is more than or equal to 1
            self.adcs = []
            for p in adc_pins:
                pin_obj = p if isinstance(p, Pin) else Pin(p)
                self.adcs.append(ADC(pin_obj))
                
                
                
                # [ADC(Pin("pin 1")), ADC(Pin("pin 2")), ADC(Pin("pin 3")), ADC(Pin("pin 4")), 
                #              ADC(Pin("pin 5")), ADC(Pin("pin 6")), ADC(Pin("pin 7")), ADC(Pin("pin 8")), 
                #              ADC(Pin("pin 9")), ADC(Pin("pin 10")), ADC(Pin("pin 11")), ADC(Pin("pin 12"))]
            self.pitch = float(pitch)
            self.oversample = max(1, int(oversample))            # Converts oversample to int, getting ta least 1 (OS of 3.5 -> 3)
            self.ema = float(ema)
            self.length = len(self.adcs)

            # Values for calibration
            self.white = [4095]*self.length           # For black, 12 bit digital range max = 4095. multiply by array of length of adc pins
            self.black = [0]*self.length              # For white, 12 bit digital range min = 0. multiply by array of length of adc pins
           
            # Position values for len(adc_pins)
            mid = (self.length-1)/2.0
            self.pos_mm = [self.pitch*(i-mid) for i in range(self.length)]           # symmetric about 0 with 4 mm pitch

            # ema filter
            self._filtered = [0.0]*self.length
            self._have_init = False
            self._last_pos = 0.0

        def _read_adc_once(self):
            return [ch.read() for ch in self.adcs]

        def _read_raw_data(self):                                        # Wire sensors left to right
            if self.oversample == 1:
                return self._read_adc_once()
            accumulator = [0]*self.length                               # Oversample, [0,0,...0]
            for _ in range(self.oversample):                            # With oversample = 4, loop runs 4 times. "_" is a throwaway variable, doesn't save, just repeats
                vals = self._read_adc_once()
                for i, v in enumerate(vals):
                    accumulator[i] += v
            return [a // self.oversample for a in accumulator]
                # v = [ADC.read() for adc in self.adc_pins]               # Reads current adc values from every sensor. v = voltage levels from each line sensor for that instant
                # accumulator = [a+b for a, b in zip(accumulator, v)]     # Pairs each total a with a new reading b and adds reading to accumulator for that sensor. This builds up sums of readings over multiple iterations to divide later to get the average
                # vals = [a // self.oversample for a in accumulator]      # Divides samples to get average. // divides and truncates
        
        def _ema_update(self, raw_vals):
            if not self._have_init:
                self._filtered = list(map(float, raw_vals))
                self._have_init = True
            else:
                a = self.ema
                for i, v in enumerate(raw_vals):
                    self._filtered[i] = a*float(v) + (1.0-a)*self._filtered[i]
            return list(self._filtered)

        def read_normalized_data(self):         # Return normalized values 0 to 1000 where white is 0 and black is 1000
            raw = self._read_raw_data()          # Get raw readings 0-4095, black-white
            filt = self._ema_update(raw)
            norm = []                           # Initialize empty list to store values
            for i, r in enumerate(filt):         # Loop through every sensor reading in raw. Enumerate gives i index and r raw value
                w = float(self.white[i])             # Calibration bounds
                b = float(self.black[i])
                if abs(b - w) < 1e-6:
                    norm.append(0)
                    continue
                if b > w:
                    t = (r - w) / (b - w)
                else:
                    t = (w -r) / (w -b)
                
                if t < 0.0: t = 0.0
                if t > 1.0: t = 1.0
                norm.append(int(round(t * 1000.0)))
            return norm
            #     if b > w:                    # Check if hi is less than lo
            #         lo, hi = 0.0, 4095.0
            #     t = (r - lo) / (hi - lo)   # Compute normalized ratio of raw reading. lo = 0, hi = 1
            #     t = 1.0 - max(0.0, min(1.0, t)) # Invert so darker is larger
            #     norm.append(int(round(t*1000.0))) # Multiply by 1000 to get in range and round to nearest integer. Add to norm list
            # return norm                         # Return completed list of norm values

        def calibrate_white(self, samples=80):                          # Reads sensors repeatedly with white surface
            mins = [65535]*self.length                                  # creates matrix with values, aka [65525, 65535 ... , 65535]
            for _ in range(max(1, samples)):                            #  Read all sensor channels directly
                v = self._read_raw_data()
                for i, x in enumerate(v):
                    if x < mins[i]:
                        mins[i] = x
            self.white = [max(0,m) for m in mins]
            print(self.white)
                # v = [ADC.read() for adc in self.adc_pins]               # Creates matrix with readings, [3910, 3965, ...]     
                # mins = [min(m,x) for m, x in zip(mins, v)]              # Keeps lowest value seen to avoid outliers
                # self.white = [max(0,m) for m in mins]                   # Stores calibration results

        def calibrate_black(self, samples=80):        
            maxs = [0]*self.length                       # creates matrix with values, aka [65525, 65535 ... , 65535]
            for _ in range(max(1, samples)):                    # Runs multiple samples                   #  Read all sensor channels directly
                v = self._read_raw_data()
                for i, x in enumerate(v):
                    if x > maxs[i]:
                        maxs[i] = x
            self.black = [min(4095, m) for m in maxs]
            print(self.black)
                # v = [ADC.read() for adc in self.adc_pins]       # Creates matrix with readings, [3910, 3965, ...]     
                # maxs = [max(m,x) for m, x in zip(maxs, v)]      # Keeps lowest value seen to avoid outliers
                # self.black = [max(4095,m) for m in maxs]           # Stores calibration results
        
        def centroid(self):
            vals = self.read_normalized_data()          # Return sensor readings normalized 1-1000 (black = 1000, white = 0)
            total = float(sum(vals))            # Sum normalized readings to get total darkness detected, float lets us use division
            if total <= 1e-9:                   # Check if total is near zero and returns zero
                return self._last_pos, 0.0
            pos = sum(v*x for v,x in zip(vals, self.pos_mm))/total      # v is normalized darkness reading from one sensor, x is physical position in mm, zip pairs each reading with its sensor position. Sum computes moment, and we divide by the total to normalize
            # pos = (sum (sensor value x sensor position)) / (sum sensor values)
            return pos, total              # Tells us sthat line is pos mm to the left(-) or right(+) of the array center
        
        def sense_line(self):
            return self.centroid()
