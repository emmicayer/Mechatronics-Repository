State Machines
====================

To organize our control over Romi, we used a state machine model with eight tasks. The tasks themselives are implimented as generator functions in separate files (other than directly in main). The are scheduled cooperatively. All tasks are kept in the global cotask.task_list and are scheduled according to their assigned priorities and periods by cotask.task_list.pri_sched() in the main loop. The eight tasks in the state machine are described below. 

.. toctree::
   :maxdepth: 1
   :caption: Contents:

   task1
   task2
   task3_4
   task5
   task6
   task7
   task8




