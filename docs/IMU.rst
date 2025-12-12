IMU Driver
=======



Our IMU driver, named BNO005 after the model of our IMU, wraps the BNO005 sensor registers and opertation modes and handles initialization, chip-ID verification, and mode changes. It includes methods to read the fused Euler angles (which includes heading, roll, and pitch) in radians, read angular velocity (which includes gyro x, y, and z) in rad/s, and read and write calibration coefficients. 

IMU Driver Code
----

.. literalinclude:: ../Source/BNO055.py
   :language: python
   :linenos:
