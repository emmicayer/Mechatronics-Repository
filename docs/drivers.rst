Hardware and Action Drivers
================

Python classes to run the motors, encoders, IR sensor, and IMU



.. toctree::
   :maxdepth: 20
   :caption: Contents:

   motor

Motor Driver
-------------

.. automodule:: motor
   :members:
   :undoc-members:
   :show-inheritance:

The file motor.py works with the motors using separate PWN and direction imputs in order to enable, disable, and drive the two motors on Romi independently. This file works with any motor inputs, ours being the following for our left and right motors:

mot_left = Motor(Pin.cpu.B1, Pin.cpu.B14, Pin.cpu.B15, Timer(3, freq = 1000, 4)) 

mot_right = Motor(Pin.cpu.B0, Pin.cpu.C3, Pin.cpu.C2, Timer(3, freq = 1000, 3))

These are two objects in the Motor class, one for each wheel. This configuration allows easy use of Motor for any combination of Romi pins and timers. These motor specification call outs are inputted into main.py for our Romi chassis and wire configuration specifically.


Encoder Driver
-------

.. automodule:: encoder
   :members:
   :undoc-members:
   :show-inheritance:

The motor class in encoder.py runs one update step on the encoder’s timer counter to keep track of the change in count of the encoder and returns the position with the most recently updated value, as well as a calculated velocity value. It also includes a definition to zero each encoder object. Our encoder objects are as follows:

enc_left = Encoder(Timer(5, freq = 1000), Pin.cpu,A0, Pin.cpu.A1) 

enc_right = Encoder(Timer(1, freq = 1000), Pin.cpu,A8, Pin.cpu.A9) 

In this driver, we have a continuous loop of updating the encoder, which allows for up to date values of the encoder to calculate the position of Romi based on values of the encoder and associated time stamps. 



Controller Driver
-----------

.. automodule:: controller
   :members:
   :undoc-members:
   :show-inheritance:

Our closed-loop velocity controller operates the two Romi motors. We chose to implement this motor in a parameterized fashion so it can be repurposed for any general PID controller use. In classical control theory, systems are seen as single-input-single-output (SISO) systems. Our controller will take a single input (an initial velocity) and output a single output (a final corrected velocity) through a PID controller, where P stands for proportional, I stands for integral, and D stands for derivative. After writing a controller outline, we tuned both our left and right controllers independently. 

Line Sensor Driver
-----------

.. automodule:: line_sensor
   :members:
   :undoc-members:
   :show-inheritance:

Our LineSensor class manages an array of ADC-based reflective sensors. It configures multiple ADC channels for the pins we provided, calibrates the sensor responses for black and white surfaces, returns normalized readings for each sensor, and computes the line centroid position in millimeters and overall line "strength" for line following control. 


IMU Driver
-----------

.. automodule:: BNO005
   :members:
   :undoc-members:
   :show-inheritance:

Our IMU driver, named BNO005 after the model of our IMU, wraps the BNO005 sensor registers and opertation modes and handles initialization, chip-ID verification, and mode changes. It includes methods to read the fused Euler angles (which includes heading, roll, and pitch) in radians, read angular velocity (which includes gyro x, y, and z) in rad/s, and read and write calibration coefficients. 


State Estimation Driver
-----------

.. automodule:: state_estimation
   :members:
   :undoc-members:
   :show-inheritance:

 Our State-estimation helper functions for Romi provides 
 kinematic relationships and utility calculations used for
 validating or interpreting estimator behavior. This driver focuses on simple analytical relationships and supporting reference calculations for estimator outputs. 
  
