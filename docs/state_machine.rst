State Machines
====================

To organize our control over Romi, we used a state machine model with eight tasks. The tasks themselives are implimented as generator functions in separate files (other than directly in main). The are scheduled cooperatively. All tasks are kept in the global cotask.task_list and are scheduled according to their assigned priorities and periods by cotask.task_list.pri_sched() in the main loop. The eight tasks in the state machine are described below. 



Task 1: User Input
----

.. automodule:: task1_UserInput
   :members: task1_UserInput
   :undoc-members:
   :show-inheritance:


This file implements a non-blocking state machine that listens for characters
over the pyboard’s USB Virtual COM Port (USB_VCP). Through simple keyboard
commands, the user can start and stop the robot, adjust left and right motor
speed setpoints, run automated test sequences, and toggle debug printing.

The task consumes and updates shared variables (motor command, motor efforts,
left/right speed setpoints) and exposes a menu-driven interface for selecting
new velocities using numeric keys. It is designed to run cooperatively inside
the scheduler, yielding control frequently while maintaining responsive input
handling.

Supported commands include:
   g : begin forward motion / automated test
   s : stop robot
   e : enter speed-selection menu
   r : choose right motor setpoint
   l : choose left motor setpoint
   0-9, t : select discrete motor speed values
   d : toggle debug output

The task ensures safe operation by interpreting commands into structured
states, updating shared data, and printing feedback messages over USB to
guide the user.

Task 2: Data Collect
----

.. automodule:: task2_DataCollect
   :members: task2_DataCollect
   :undoc-members:
   :show-inheritance:

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


Task 3: Left Control
----

.. automodule:: task3_Control
   :members: task3_Control
   :undoc-members:
   :show-inheritance:

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

Task 3 specializes in the left motor, and is an object of the encoder and motor classses that is put into the general control task. 


Task 4: Right Control
----

.. automodule:: task3_Control
   :members: task3_Control
   :undoc-members:
   :show-inheritance:

Task 4 control uses the same file as task3_Control but uses a different task object to code for the right motor as opposed to the left motor. The same documentation still follows. 





Task 5: State Estimation
----

.. automodule:: task5_StateEstimation_DT
   :members: task5_StateEstimation_DT
   :undoc-members:
   :show-inheritance:


This task implements a discrete observer, updated at each scheduler
cycle, using precomputed Ad and Bd matrices derived from a MATLAB c2d model.
It reads:
   • Motor input voltages (u) from the input queue
   • Wheel displacements and IMU measurements (y) from the measurement queue

Using these, it computes:
   • Estimated forward velocity
   • Estimated yaw rate
   • Integrated heading (theta)
These are published to shared variables for use by navigation, diagnostics,
and higher-level control.

The task handles missing data, automatically computes sampling
intervals using `micros()`, and prints debug information periodically. It
runs only while Romi is active, yielding often to coexist with other
scheduler tasks.

Task 6: Bump Sensor
----

.. automodule:: task6_BumpSensor
   :members: task6_BumpSensor
   :undoc-members:
   :show-inheritance:

This file defines the BumpSensors class that configures GPIO inputs
with pull-ups to ensure clean transitions.
Each sensor corresponds to a bit in the mask, allowing simultaneous detection
of multiple pressed switches.

The TaskBump function continuously reads the sensors, updates the shared
bump-mask variable, and optionally prints changes over USB for debugging.
Other tasks (such as motor control) monitor this mask to trigger an immediate
stop for collision safety.

The task yields frequently and is designed to run continuously within the
cooperative scheduler.


   
Task 7: Bluetooth Control
----

.. automodule:: task7_BluetoothControl
   :members: task7_BluetoothControl
   :undoc-members:
   :show-inheritance:

 This task processes incoming text-based commands sent over Bluetooth and
 updates shared variables accordingly. Supported functions include:
    • Starting and stopping the robot (GO / STOP)
    • Setting left and right velocity setpoints (VEL L=… R=…)
    • Changing control modes (MANUAL / LINE)
    • Enabling or disabling telemetry streaming
    • Returning robot status (GET STATUS)
    • Basic connection testing (PING, which returns PONG)

 Commands are parsed using a lightweight line reader with buffer management.
 When telemetry is enabled, the task periodically transmits motor setpoints,
 mode, bump-mask status, and motor command state.

 The task runs as a non-blocking state machine, yielding frequently to maintain
 responsiveness alongside other system tasks.


Task 8: Track Run
----

.. automodule:: task8_TrackRun
   :members: task8_TrackRun
   :undoc-members:
   :show-inheritance:

TrackRun is a high level script that sections the game course into chunks to separate them into line following sections and state estimation sections. 
