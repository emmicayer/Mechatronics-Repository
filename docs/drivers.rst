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


Line Sensor Driver
-----------

.. automodule:: line_sensor
   :members:
   :undoc-members:
   :show-inheritance:

Our LineSensor class manages an array of ADC-based reflective sensors. It configures multiple ADC channels for the pins we provided, calibrates the sensor responses for black and white surfaces, returns normalized readings for each sensor, and computes the line centroid position in millimeters and overall line "strength" for line following control. 




State Estimation Driver
-----------

.. automodule:: state_estimation
   :members:
   :undoc-members:
   :show-inheritance:

 Our State-estimation helper functions for Romi provides 
 kinematic relationships and utility calculations used for
 validating or interpreting estimator behavior. This driver focuses on simple analytical relationships and supporting reference calculations for estimator outputs. 
  
