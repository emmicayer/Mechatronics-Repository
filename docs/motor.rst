Motor Driver
=====

The file motor.py works with the motors using separate PWN and direction imputs in order to enable, disable, and drive the two motors on Romi independently. This file works with any motor object inputs. 

These are two objects in the Motor class, one for each wheel. This configuration allows easy use of Motor for any combination of Romi pins and timers. These motor specification call outs are inputted into main.py for our Romi chassis and wire configuration specifically.


Motor Driver Code
----

.. literalinclude:: ../Source/motor.py
   :language: python
   :linenos:
