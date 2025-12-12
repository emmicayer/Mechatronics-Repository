Main
=====

Main is a top-level script which configures and runs all cooperative tasks. This file initializes the BNO055 IMU and restores (or generates) its
calibration data, then creates all the Share and Queue objects used
for inter-task communication. It instantiates the individual task functions
from the other modules as otask.Task objects, appends them to
cotask.task_list , and starts the cooperative scheduler with
cotask.task_list.pri_sched(). The tasks themselves are implemented as generator functions in other
modules and are scheduled cooperatively. All tasks are kept in the global cotask.task_list and are scheduled
according to their assigned priorities and periods by
cotask.task_list.pri_sched() in the main loop.


Main Code
----
.. literalinclude:: ../Source/main.py
   :language: python
   :linenos:
