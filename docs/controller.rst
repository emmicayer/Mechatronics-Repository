Controller Driver
=====


Our closed-loop velocity controller operates the two Romi motors. We chose to implement this motor in a parameterized fashion so it can be repurposed for any general PID controller use. In classical control theory, systems are seen as single-input-single-output (SISO) systems. Our controller will take a single input (an initial velocity) and output a single output (a final corrected velocity) through a PID controller, where P stands for proportional, I stands for integral, and D stands for derivative. After writing a controller outline, we tuned both our left and right controllers independently. 

Controller Driver Code
----
.. literalinclude:: ../Source/controller.py
   :language: python
   :linenos:
