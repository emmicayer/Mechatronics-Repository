Task 6: Bump Sensor
====


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

Bump Sensor Code
----

.. literalinclude:: ../Source/task6_BumpSensor.py
   :language: python
   :linenos:
