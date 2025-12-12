Motor Driver
=====

The file motor.py works with the motors using separate PWN and direction imputs in order to enable, disable, and drive the two motors on Romi independently. This file works with any motor object inputs. 

mot_left = Motor(Pin.cpu.B1, Pin.cpu.B14, Pin.cpu.B15, Timer(3, freq = 1000, 4)) 

mot_right = Motor(Pin.cpu.B0, Pin.cpu.C3, Pin.cpu.C2, Timer(3, freq = 1000, 3))

These are two objects in the Motor class, one for each wheel. This configuration allows easy use of Motor for any combination of Romi pins and timers. These motor specification call outs are inputted into main.py for our Romi chassis and wire configuration specifically.

.. literalinclude:: ../Source/motor.py
   :language: python
   :linenos:
