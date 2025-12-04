import pyb
from pyb import UART
from ulab import numpy as np
import math


# TASK 2 - data collection

def DataCollect(shares):     
    s_mot_cmd, q_pos_L, q_vel_L, q_time_L, q_pos_R, q_vel_R, q_time_R, s_pos_L, s_vel_L, s_time_L, s_mot_eff_L, s_pos_R, s_vel_R, s_time_R, s_mot_eff_R, imu, q_u, q_y = shares        
    state = 1
    BT = UART(1, 115200)
    r_wheel = 0.035
    CPR_L = 5748.5        # Counts per revolution
    CPR_R = 5748.5        # Counts per revolution
    v_bat = 7.2
    K   = (250.0 * 2.0 * 3.14159265 / 60.0) / 4.5   # Motor gain [rad/(v*s)]
    tau = 0.1                                       #s Motor time constant [s]
    last_t = pyb.micros()
    last_omega_L = 0.0
    last_omega_R = 0.0
    first_init_done = False
    w = 0.141  # Tract width [m]
    next_print_time = pyb.millis() + 1000  # print once every 1000 ms

 
    sL_m = 0.0
    sR_m = 0.0
    last_pos_L = s_pos_L.get()
    last_pos_R = s_pos_R.get()

    while True:
        if s_mot_cmd.get() == 1.0:
            state = 2
        if state == 1:              # idle
            if s_mot_cmd.get() == 1.0:
           
                # motor is turned on and should be collecting data 

                state = 2           # Set to next state
            yield 0 
                    
        elif state == 2:            # Buffer, collecting data
        
            if not first_init_done:
                try:
                    psi0, _, _ = imu.read_euler_angles()   # Radians
                    psi0 = (psi0 + math.pi) % (2*math.pi) - math.pi
                except Exception:
                    psi0 = 0.0

                # Initialize integrated wheel displacements so encoders "match" IMU heading
                sR_m = -(w / 2.0) * psi0
                sL_m = (w / 2.0) * psi0

                # Re-baseline the raw encoder positions to avoid a big first delta
                last_pos_L = s_pos_L.get()
                last_pos_R = s_pos_R.get()

                first_init_done = True

            pos_L = s_pos_L.get()
            vel_L = s_vel_L.get()
            time_L = s_time_L.get()
            pos_R = s_pos_R.get()
            vel_R = s_vel_R.get()
            time_R = s_time_R.get()

            if not q_pos_L.full():  q_pos_L.put(pos_L)
            if not q_vel_L.full():  q_vel_L.put(vel_L)
            if not q_time_L.full(): q_time_L.put(time_L)

            if not q_pos_R.full():  q_pos_R.put(pos_R)
            if not q_vel_R.full():  q_vel_R.put(vel_R)
            if not q_time_R.full(): q_time_R.put(time_R)

            # Estimator input/output vectors
            now_t = pyb.micros()
            dt_us = (now_t - last_t) & 0xFFFFFFFF
            last_t = now_t
            if dt_us <= 0:
                dt_us = 10000
            dt_us = dt_us*1e-6

            # Encoder deltas
            dL = pos_L - last_pos_L;  last_pos_L = pos_L
            dR = pos_R - last_pos_R;  last_pos_R = pos_R

            # Convert to angular velocities [rad/s]
            omega_L = (dL / dt_us) * 2*3.1415/CPR_L
            omega_R = (dR / dt_us) * 2*3.1415/CPR_R

            # Numeric derivatives [rad/s^2]
            domega_L = (omega_L - last_omega_L) / dt_us
            domega_R = (omega_R - last_omega_R) / dt_us
            last_omega_L = omega_L
            last_omega_R = omega_R

            # Estimate motor voltages using first-order model
            # u ≈ (tau * domega + omega) / K
            # vL = (tau * domega_L + omega_L) / K
            # vR = (tau * domega_R + omega_R) / K

            effL = float(s_mot_eff_L.get())
            effR = float(s_mot_eff_R.get())
            vL = 0.01 * effL * v_bat
            vR = 0.01 * effR * v_bat
         
            # effL = float(s_mot_eff_L.get())
            # effR = float(s_mot_eff_R.get())
            # vL = 0.01*effL*v_bat
            # vR = 0.01*effR*v_bat

            try: 
                while q_u.any():
                    q_u.get()
            except AttributeError:
                pass
            q_u.put(vL)
            q_u.put(vR)

            # dL = pos_L - last_pos_L
            # last_pos_L = pos_L
            # dR = pos_R - last_pos_R
            # last_pos_R = pos_R

            sL_m += (2*3.1415*r_wheel/CPR_L)*dL
            sR_m += (2*3.1415*r_wheel/CPR_R)*dR


            # IMU heading [rad] and yaw rate [rad/s]
            try:
                theta, roll, pitch = imu.read_euler_angles()
                gx, gy, gz = imu.read_angular_velocity()
            except Exception:
                theta, gz = 0.0, 0.0

            try:
                while q_y.any():
                    q_y.get()
            except AttributeError:
                pass
            q_y.put(sL_m)
            q_y.put(sR_m)
            q_y.put(theta)
            q_y.put(gz)

            if pyb.millis() >= next_print_time:
                try:
                    print("DC u=[{:.3f},{:.3f}] y=[sL={:.4f}, sR={:.4f}, th={:.3f}, gz={:.3f}]"
                        .format(vL, vR, sL_m, sR_m, theta, gz))
                except Exception:
                    pass
                next_print_time = pyb.millis() + 1000

            # linL = r_wheel * omega_L
            # linR = r_wheel * omega_R
            # speed_thresh = 0.002   # m/s, tune threshold for "moving"
            # gyro_thresh  = 0.02    # rad/s, yaw-rate threshold

            # is_moving = (abs(linL) > speed_thresh) or (abs(linR) > speed_thresh) or (abs(gz) > gyro_thresh)

            # if is_moving and pyb.millis() >= next_print_time:
            #     try:
            #         print("DC u=[{:.3f},{:.3f}] y=[sL={:.4f}, sR={:.4f}, th={:.3f}, gz={:.3f}]"
            #             .format(vL, vR, sL_m, sR_m, theta, gz))
            #     except Exception:
            #         pass
            #     next_print_time = pyb.millis() + 1000

            # Bluetooth steaming of data collection
            line = "{},{},{},{},{},{}\r\n".format(time_L, pos_L, vel_L, time_R, pos_R, vel_R)
            try:
                BT.write(line.encode())
            except:
                pass

            if s_mot_cmd.get() == 0.0:
                state = 1
                yield 0

            if s_mot_cmd.get() == 0.0 and abs(vL) < 0.01 and abs(vR) < 0.01:
                state = 1

            yield 0


