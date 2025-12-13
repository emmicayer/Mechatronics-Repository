.. ME 405 Final Project documentation master file, created by
   sphinx-quickstart on Wed Dec 10 20:14:16 2025.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Mechatronics Term Project Documentation
================================

Project Overview
----

The final product of this project was to complete a line following and state estimation game track with Romi hardware. Our code is outlined using a priority sceduler with each task as a cooperative finite state machine. In our main file, a scheduler is run continuously with each task as a finite state machine. This main file initializes all tasks, shares, queues, and class objects. We have two main sections of code: drivers and tasks. We have six drivers: Motor Driver, Encoder Driver, Controller Driver, Line Sensor Driver, IMU Driver, and State Estimation Driver. Each driver is organized as a class that can take in objects and object arguments. More information about each specifc driver can be found below in the contents. We have eight tasks overall, with two of them being a part of the same code file (control left and control right). Documentation for these files can also be found below.

Below are the contents for our designed hardware, drivers, state machines, and relevent diagrams and plots for the control of Romi for the Fall 2025 Mechatronics game track. All written code and documentation was completed by Emmi Cayer and Erin Maxwell. The following links to the Respository which includes offical code, diagram, and plot files: https://github.com/emmicayer/Mechatronics-Repository, but the code utilized in our game track solution is also outlined below in full. 


.. toctree::
   :maxdepth: 2
   :caption: Contents:

   main
   hardware
   drivers
   statemachine
   taskandstatediagrams
   plots
   supportcode
   pininfo
   videos



Game Track
----

This is the game track that Romi was tasked to navigate. This game track includes multiple sections that can be navigated purely with line following, a split path, checkpoints, a square wave section, a garage, and a wall to hit and navigate around. To complete this task, we section off the track into half line following sections and half state estimation sections. 

.. figure:: images/gametrack.png
   :align: center
   :height: 300px
   :width: 600px
   :alt: alternate text


