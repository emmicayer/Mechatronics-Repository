Mechatronics Repository README
============

This file outlines the organization of Emmi Cayer and Erin Maxwell's Mechatronics Repository. 

The folder labeled "docs" includes all files for the implimentation of readthedocs, which is a comprehensive website outlining our task organization, hardware, and drivers. This website can be found at: https://mechatronics-repository.readthedocs.io/.

The folder labeled "Source" holds all source code used to impliment Romi's behavior on the track. These include driver files, task files, and other support files. The driver files included in our repository are:
* 'controller.py'
* 'encoder.py'
* 'encoder.py'
* 'line_sensor.py'
* 'motor.py'
* 'state_estimation.py'

Next, we have our task files. These are the tasks implimented as finite state machines that act in a cooperative multitasking fashion through 'main'. 
* 'task1_UserInput.py'
* 'task2_DataCollect.py'
* 'task3_Control.py', which includes tasks 3 and 4 as objects
* 'task5_StateEstimation_DT'
* 'task6_BumpSensor'
* 'task7_BluetoothControl'
* 'task8_TrackRun'

In our source files, we've also included some support files that were used to test and impliment our code throughout the process of this journey with Romi. These files are not necessary to run Romi, but are included to show our whole implimentation process. These files include:
* 'PC_script.py'
* 'data.csv'

Other support files used to impliment cooperative multitasking in Python are shown in a separate repository here: https://github.com/spluttflob/ME405-Support. These are written by the user spluttflob. 
* 'cotask.py'
* 'task_share.py'

These files are all called out and utilized in 'main.py' through a task sceduler outlined with spluttflob's support files. 
