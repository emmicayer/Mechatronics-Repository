

Hardware
=============


IMU
----

.. automodule:: IMU_I2C
   :members: IMU_I2C
   :undoc-members:
   :show-inheritance:

To track Romi's position, heading, and velocity, we used a BNO055. The BNO055 is a 9-axis intertial measurement unit (IMU) to provide data on Romi orientation. The sensor can output raw data from the accelerometer, gyroscope, and magnetometer, each in 3 axes. We used specifically the heading, or yaw angle, provided by the sensor since this data is not directly conveyed by the encoders on the wheels. We proved that the output of this value on the IMU was correct with physical testing on a jackstand that pivoted as needed. This IMU heading value provides a better data set that is drift-free and gets rid of any effect of slip on encoder values. 


IR Sensor Array
---------------

.. automodule:: sensor_array
   :members:
   :undoc-members:
   :show-inheritance:

For our line sensor, we decided to use the sensors found on the Pololu website: https://www.pololu.com/product/4213. These are 4 mm X 13 line sensors with a max board current of220 mA and a max range of 40 mm. They are the analog output type, which allows for a continuous and detailed data output signal that is directly proportional to the physical quantity being measured. Their formal name on the Pololu website is QTR-HD-13A. 
