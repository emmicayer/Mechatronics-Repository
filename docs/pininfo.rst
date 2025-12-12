Romi Pinout and Wiring Diagram
=================



A pinout is a complete wiring diagram system that maps all electrical connections on Romi. It shows which pins correspond to motors, encoders, sensors, serial interfaces, power rails, and expansion headers in order to keep wiring correct and consistent. 

We chose this pin configuration based on Nucleo pin data and to optimize and minimize distances between similar pins (aka left motor pins are optimially in a similar section). This pinout matters because it ensures correct wiring during Romi assembly, prevents short circuits, and helps us debug code errors related to pins quickly. This also allows us to keep track of pins for driver objects. 



Romi Pinout
----

.. figure:: images/pinout.png
   :align: center
   :height: 300px
   :width: 600px
   :alt: alternate text


Romi Pin Header Detail
----

.. figure:: images/headerdetail.png
   :align: center
   :height: 500px
   :alt: alternate text

Romi Wiring Harness
----

.. figure:: images/wiringharness.png
   :align: center
   :height: 500px
   :alt: alternate text
