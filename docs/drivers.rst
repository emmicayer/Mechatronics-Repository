Hardware and Action Drivers
================

Python classes to run the motors, encoders, IR sensor, and IMU



.. toctree::
   :maxdepth: 20
   :caption: Contents:

   motor
   encoder
   controller
   line_sensor
   IMU
   state_estimation





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
  
