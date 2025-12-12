Task 5: State Estimation
=====


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

State Estimation Code
----

.. literalinclude:: ../Source/task5_StateEstimation_DT.py
   :language: python
   :linenos:
