Tasks 3 and 4: Left Control and Right Control
=====


This file sets up motor drivers, encoders, PID controllers, and a line sensor
array. It provides two main tasks, `run_L` and `run_R`, which independently
control each wheel in a cooperative multitasking environment. Each task:

  • Enables/disables its motor based on the global motor command signal
  • Reads encoder position and velocity to update shared measurement values
  • Runs a velocity PID loop to drive the wheel toward its commanded setpoint
  • Publishes motor effort outputs for use by logging and state estimation
  • Monitors bump sensors to immediately stop the motors for safety

The file also incorporates line-sensor calibration and a steering PID
controller. When enabled, this steering loop adjusts left/right setpoints
to maintain the robot centered over a detected line.

These control tasks execute continuously while the robot is active, using
frequent yields to integrate seamlessly with the rest of the scheduler.

Task 3 specializes in the left motor, and is an object of the encoder and motor classses that is put into the general control task. Task 4 control uses the same file as task3_Control but uses a different task object to code for the right motor as opposed to the left motor. The same documentation still follows. 

Left Control Code
----

.. literalinclude:: ../Source/task3_Control.py
   :language: python
   :linenos:
