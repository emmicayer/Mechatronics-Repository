Encoder Driver
=====


The motor class in encoder.py runs one update step on the encoder’s timer counter to keep track of the change in count of the encoder and returns the position with the most recently updated value, as well as a calculated velocity value. It also includes a definition to zero each encoder object. We focus on two encoder objects (one for each Romi wheel) but this driver could be extended to any encoder object. 

In this driver, we have a continuous loop of updating the encoder, which allows for up to date values of the encoder to calculate the position of Romi based on values of the encoder and associated time stamps. 

Encoder Driver Code
----

.. literalinclude:: ../Source/encoder.py
   :language: python
   :linenos:
