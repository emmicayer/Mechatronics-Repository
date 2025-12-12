.. ME 405 Final Project documentation master file, created by
   sphinx-quickstart on Wed Dec 10 20:14:16 2025.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Mechatronics Term Project Documentation
================================

This homepage outlines the hardware, drivers, state machine, and relevent diagrams and plots for the control of Romi for the Fall 2025 Mechatronics game track. All written code and documentation was completed by Emmi Cayer and Erin Maxwell. 


.. toctree::
   :maxdepth: 20
   :caption: Contents:


   hardware
   drivers
   state_machine
   taskandstatediagrams
   plots


Game Track
----

This is the game track that Romi was tasked to navigate. This game track includes multiple sections that can be navigated purely with line following, a split path, checkpoints, a square wave section, a garage, and a wall to hit and navigate around. In our `task8_TrackRun.py`, we section off the track into half line following sections and half state estimation sections. 

.. figure:: images/gametrack.png
   :align: center
   :height: 300px
   :width: 600px
   :alt: alternate text
