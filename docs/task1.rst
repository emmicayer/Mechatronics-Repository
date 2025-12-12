Task 1: User Input
=====

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


User Input Code
----

.. literalinclude:: ../Source/task1_UserInput.py
   :language: python
   :linenos:
