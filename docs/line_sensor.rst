Line Sensor Driver
=====



Our LineSensor class manages an array of ADC-based reflective sensors. It configures multiple ADC channels for the pins we provided, calibrates the sensor responses for black and white surfaces, returns normalized readings for each sensor, and computes the line centroid position in millimeters and overall line "strength" for line following control. 

Line Sensor Driver Code
----

.. literalinclude:: ../Source/line_sensor.py
   :language: python
   :linenos:
