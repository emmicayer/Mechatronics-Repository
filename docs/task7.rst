Task 7: Bluetooth Control
====


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

Bluetooth Control Code
----

.. literalinclude:: ../Source/task7_BluetoothControl.py
   :language: python
   :linenos:
