## @file main.py
#  @brief Top-level script which configures and runs all cooperative tasks.
#
#  This file initializes the BNO055 IMU and restores (or generates) its
#  calibration data, then creates all the @c Share and @c Queue objects used
#  for inter-task communication. It instantiates the individual task functions
#  from the other modules as @c cotask.Task objects, appends them to
#  @c cotask.task_list , and starts the cooperative scheduler with
#  @c cotask.task_list.pri_sched().
#
#  The tasks themselves are implemented as generator functions in other
#  modules and are scheduled cooperatively:
#  @li @c UserInput          – Reads user commands and updates motor commands
#                              and target velocities.
#  @li @c DataCollect        – Logs motor position, velocity, time, and IMU
#                              data into queues and shares.
#  @li @c run_L / run_R      – Closed-loop control tasks for the left and
#                              right motors, using shared state and commands.
#  @li @c StateEstimation    – Uses logged input/output data to estimate
#                              the robot state (e.g., @c x_hat and @c y_hat ).
#  @li @c TaskBump           – Monitors bump sensors and updates a bump
#                              bitmask share used by other tasks.

#  All tasks are kept in the global @c cotask.task_list and are scheduled
#  according to their assigned priorities and periods by
#  @c cotask.task_list.pri_sched() in the main loop.

import gc
import pyb
from pyb import Timer, Pin, delay, I2C
from time import sleep_ms
import cotask
import task_share
from motor import Motor			        # imports Motor class
from encoder import Encoder		        # imports Encoder class
from task1_UserInput import UserInput
from task2_DataCollect import DataCollect
from task3_Control import run_L, run_R
from task5_StateEstimation_DT import StateEstimation
from controller import Controller       # imports Controller class
from line_sensor import LineSensor
from BNO055 import BNO055
from task6_BumpSensor import TaskBump
import os
from task7_BluetoothControl import BluetoothTask
#from task8_TrackRun import TrackRun

# Initialize IMU, read calibration data from file

print("Initializing IMU")
i2c = I2C(1, I2C.CONTROLLER, baudrate=400000)       # Creates i2c object for bus 1 on nucleo, board acts as controller. i2c clock 400 khz
imu = BNO055(i2c)

try:                                                
    with open('/flash/calibration.txt', 'rb') as f: # Try to open calibration file, rb = read binary
        imu_data = f.read()                         # Reads 22 bytes of calibration coefficients
    imu.write_cal_coeff(imu_data)                   # Writes bytes into BMO055's internal calibration registers
    print("IMU calibration data restored") 
    imu.change_operating_mode(imu.mode_ndof)
    pyb.delay(20)         
except Exception as e:                              # No file found, if any error inside try, call error object e
    print("No callibration data found, starting calibration:", e)   # Print no calibration data found followed by system error message

    imu.change_operating_mode(imu.mode_config)
    delay(20)
    imu.i2c.mem_write(bytes([0]*imu.calib_len), imu.addr, imu.reg_calib_start)
    delay(20)
    imu.change_operating_mode(imu.mode_ndof)        # Enter fusion mode so sensor calibrates
    delay(20)                                       # Wait 19+ seconds for IMU to update according to data sheet


    fully_calibrated = False
    for i in range(10000):                          # Loops 40 times max, which is about 20 seconds (40*500 ms)
        stat = imu.retrieve_cal_status()            # Reads one byte from register 0x35 that encodes calibration levels
        sys = (stat >> 6) & 0x03                    # Bits 7-6 extracted with bit shifting and masking
        gyr = (stat >> 4) & 0x03                    # Bits 5-4
        acc = (stat >> 2) & 0x03                    # Bits 3-2
        mag = stat & 0x03                           # Bits 1-0
        print("cal -> sys:{} gyr:{} acc:{} mag:{}".format(sys, gyr, acc, mag))  # Prints current calibration levels so we can watch them change
        delay(500)                                  # Wait 0.5 s between checks
        if sys == 3 and gyr == 3 and acc == 3 and mag == 3: # If all values are 3 (fully calibrated), break the loop
            fully_calibrated = True
            break

    if fully_calibrated:
        cal = imu.read_cal_coeff()                      # Read 22 byte calibration data from IMU's registers
        with open('/flash/calibration.txt', 'wb') as f: # Opens file for wb (writing in binary) 
            f.write(cal)                                # Writes all bytes to the file so they stay there
        print("Calibration complete and saved to calibration.txt")  
    else:
        print("Calibration not complete error")

    imu.change_operating_mode(imu.mode_ndof)            # Change back to fusion mode, puts IMU back in ndof mode so it produces real data
    print("IMU ready for use")


# Main
if __name__ == "__main__":

    # Shares
    s_mot_eff_L = task_share.Share('f', thread_protect=False, name="Left Motor Effort Share")
    s_mot_eff_R = task_share.Share('f', thread_protect=False, name="Right Motor Effort Share")
    s_mot_cmd = task_share.Share('B', thread_protect=False, name="MLeft otor Command Share")
    s_pos_L = task_share.Share('f', thread_protect=False, name="Left Position Share")
    s_vel_L = task_share.Share('f', thread_protect=False, name="Left Velocity Share")
    s_time_L = task_share.Share('f', thread_protect=False, name="Left Time Share")
    s_pos_R = task_share.Share('f', thread_protect=False, name="Right Position Share")
    s_vel_R = task_share.Share('f', thread_protect=False, name="Right Velocity Share")
    s_time_R = task_share.Share('f', thread_protect=False, name="Right Time Share")
    s_new_setpoint_L = task_share.Share('f', thread_protect=False, name="Left Target Velocity Value")
    s_new_setpoint_R = task_share.Share('f', thread_protect=False, name="Right Target Velocity Value")
    s_mot_eff = task_share.Share('f', thread_protect=False, name="Motor Effort Share")
    s_shat = task_share.Share('f', thread_protect=False, name="s hat Share")
    s_psihat = task_share.Share('f', thread_protect=False, name="psi hat Share")
    s_bump_mask = task_share.Share('B', thread_protect=False, name="Bump Bitmask Share")
    s_track_section = task_share.Share('B', thread_protect=False, name="Track Section Share")
    s_line_follow_en = task_share.Share('B', thread_protect=False, name="Line Follow Enable Share")

    # Queues
    q_pos_L = task_share.Queue('f', 100, thread_protect=False, overwrite=False, name="Left Position Queue")
    q_vel_L = task_share.Queue('f', 100, thread_protect=False, overwrite=False, name="Left Velocity Queue")
    q_time_L = task_share.Queue('f', 100, thread_protect=False, overwrite=False, name="Left Time Queue")
    q_pos_R = task_share.Queue('f', 100, thread_protect=False, overwrite=False, name="Right Position Queue")
    q_vel_R = task_share.Queue('f', 100, thread_protect=False, overwrite=False, name="Right Velocity Queue")
    q_time_R = task_share.Queue('f', 100, thread_protect=False, overwrite=False, name="Right Time Queue")
    q_u = task_share.Queue('f', 100, thread_protect=False, overwrite=True, name="Input Vector Queue")
    q_y = task_share.Queue('f', 100, thread_protect=False, overwrite=True, name="Output Vector Queue")

    # Task creation 
    task1 = cotask.Task(UserInput, name="UserInput", priority=0, period=0,                        # creating task1 object of cotask Task class. tasks are objects in main
                        profile=True, trace=True, shares=(s_mot_cmd, s_mot_eff_R, s_mot_eff_L, s_new_setpoint_L, s_new_setpoint_R))         # put shares and queues under shares=(,)
    task2 = cotask.Task(DataCollect, name="DataCollect", priority=2, period=25,
                        profile=True, trace=True, shares=(s_mot_cmd, q_pos_L, q_vel_L, q_time_L, q_pos_R, q_vel_R, q_time_R, s_pos_L, s_vel_L, s_time_L, s_mot_eff_L, s_pos_R, s_vel_R, s_time_R, s_mot_eff_R, imu, q_u, q_y))
    task3 = cotask.Task(run_L, name="LeftControl", priority=3, period=20,
                        profile=True, trace=True, shares=(s_mot_cmd, s_mot_eff_L,s_pos_L, s_vel_L, s_time_L, s_new_setpoint_L, s_new_setpoint_R, s_bump_mask, s_line_follow_en))
    task4 = cotask.Task(run_R, name="RightControl", priority=3, period=20,
                        profile=True, trace=True, shares=(s_mot_cmd, s_mot_eff_R, s_pos_R, s_vel_R, s_time_R, s_new_setpoint_R, s_new_setpoint_L, s_bump_mask, s_line_follow_en))
    task5 = cotask.Task(StateEstimation, name="StateEstimation", priority=1, period=20,
                        profile=True, trace=True, shares=(s_mot_cmd, s_mot_eff_L, s_mot_eff_R, s_new_setpoint_L, s_new_setpoint_R, s_shat, s_psihat, q_u, q_y))
    task6 = cotask.Task(TaskBump, name="Bump", priority=4, period=10,
                        profile =True, trace = True, shares=(s_bump_mask,))
    task7 = cotask.Task(BluetoothTask, name="Bluetooth", priority=4, period=10,
                        profile=True, trace=True, shares=(s_mot_cmd, s_new_setpoint_L, s_new_setpoint_R, s_bump_mask) )
<<<<<<< Updated upstream
    #task8 = cotask.Task(TrackRun, name="TrackRun", priority=2, period=20,
    #                    profile=True, trace=True, shares=(s_pos_L, s_pos_R, s_xhat, s_yhat, s_track_section, s_line_follow_en, s_new_setpoint_L, s_new_setpoint_R, s_mot_cmd))
=======
    task8 = cotask.Task(TrackRun, name="TrackRun", priority=2, period=20, 
                        profile=True, trace=True, shares=(s_pos_L, s_pos_R, s_xhat, s_yhat, s_track_section, s_line_follow_en, s_new_setpoint_L, s_new_setpoint_R, s_mot_cmd, s_bump_mask,))
>>>>>>> Stashed changes

    # Add tasks to task list
    cotask.task_list.append(task1)
    cotask.task_list.append(task2)
    cotask.task_list.append(task3)
    cotask.task_list.append(task4)
    cotask.task_list.append(task5)
    cotask.task_list.append(task6)
    cotask.task_list.append(task7)   
    #cotask.task_list.append(task8)

    # Run the memory garbage collector
    gc.collect()

    # Run the scheduler
    while True:
        try:
            cotask.task_list.pri_sched()
        except KeyboardInterrupt:
            print(cotask.task_list)
            break
