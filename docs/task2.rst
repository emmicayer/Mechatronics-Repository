Task 2: Data Collect
=====


This task runs whenever the motor command indicates that the robot is active.
It reads encoder positions, velocities, timestamps, and IMU heading/yaw-rate
data, then computes wheel displacements, wheel angular velocities, and
estimated motor voltages using a simplified motor model. These values are
placed into FIFO queues for later processing by the state estimator and for
Bluetooth telemetry.

Additional features include:
  • Automatic initialization of encoder-based displacement tracking using IMU
    orientation so wheel odometry aligns with the measured heading
  • Calculation of wheel displacements (meters) by integrating encoder ticks
  • Periodic printing of diagnostic values over USB
  • Streaming raw encoder and velocity data over Bluetooth for live plotting

The task operates continuously while the robot is moving and yields frequently
to cooperate within the system scheduler.

Data Collect Code
----

.. literalinclude:: ../Source/task2_DataCollect.py
   :language: python
   :linenos:
