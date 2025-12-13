

Hardware
=============

Nucleo-64
----

We used a Nucleo-64 board from ST Microelectronics to complete the task of a line-following and state-estimating Romi. To do this, we employed a simple custom board called a Shoe of Brian which sits below the Nucleo and houses a USB connector. This USB connector allows for mounting some of the microcontroller's flash memory as a USB file system. This microcontroller board comes with pins to configure as GPIO, timers, and many other purposes. Code is flashed through the USB to this microcontroller for easy programming updates. 

IMU
----

To track Romi's position, heading, and velocity, we used a BNO055. The BNO055 is a 9-axis intertial measurement unit (IMU) to provide data on Romi orientation. The sensor can output raw data from the accelerometer, gyroscope, and magnetometer, each in 3 axes. We used specifically the heading, or yaw angle, provided by the sensor since this data is not directly conveyed by the encoders on the wheels. We proved that the output of this value on the IMU was correct with physical testing on a jackstand that pivoted as needed. This IMU heading value provides a better data set that is drift-free and gets rid of any effect of slip on encoder values. 


IR Sensor Array
---------------

For our line sensor, we decided to use the sensors found on the Pololu website: https://www.pololu.com/product/4213. These are 4 mm X 13 line sensors with a max board current of220 mA and a max range of 40 mm. They are the analog output type, which allows for a continuous and detailed data output signal that is directly proportional to the physical quantity being measured. Their formal name on the Pololu website is QTR-HD-13A. 

Bump Sensors
---------------

For our bump sensors, we decided to use Pololu bump sensors found here: https://www.pololu.com/product/3674. There is one sensor for the right and one for the left. We have these set up in a mask array for each side, which automatically triggers a motor off command. 


Other Components
----

In addition to the IMU and IR Sensor Array, there were a few other components that we needed for assembly. These are outlined below. 

.. figure:: images/components.png
   :align: center
   :height: 500px
   :alt: alternate text
