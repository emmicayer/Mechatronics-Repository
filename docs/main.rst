Main
=====
@brief Top-level script which configures and runs all cooperative tasks.

This file initializes the BNO055 IMU and restores (or generates) its
calibration data, then creates all the @c Share and @c Queue objects used
for inter-task communication. It instantiates the individual task functions
from the other modules as @c cotask.Task objects, appends them to
@c cotask.task_list , and starts the cooperative scheduler with
@c cotask.task_list.pri_sched().

The tasks themselves are implemented as generator functions in other
modules and are scheduled cooperatively:
@li @c UserInput          – Reads user commands and updates motor commands
                            and target velocities.
@li @c DataCollect        – Logs motor position, velocity, time, and IMU
                             data into queues and shares.
@li @c run_L / run_R      – Closed-loop control tasks for the left and
                            right motors, using shared state and commands.
@li @c StateEstimation    – Uses logged input/output data to estimate
                            the robot state (e.g., @c x_hat and @c y_hat ).
@li @c TaskBump           – Monitors bump sensors and updates a bump
                            bitmask share used by other tasks.

All tasks are kept in the global @c cotask.task_list and are scheduled
according to their assigned priorities and periods by
@c cotask.task_list.pri_sched() in the main loop.


Main Code
----
.. literalinclude:: ../Source/main.py
   :language: python
   :linenos:
