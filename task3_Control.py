import pyb
from pyb import Timer, Pin
from motor import Motor
from encoder import Encoder
from controller import Controller 
from line_sensor import LineSensor

# TASK 3 - encoder, CLMC, motor

TIM2 = Timer(2, freq = 1000)

mot_left = Motor(Pin.cpu.B10, Pin.cpu.B14, Pin.cpu.B15, TIM2, 3)
mot_right = Motor(Pin.cpu.B11, Pin.cpu.B2, Pin.cpu.B12, TIM2, 4)
enc_left = Encoder(Timer(3, freq = 5000), Pin.cpu.B4, Pin.cpu.B5)
enc_right = Encoder(Timer(1, freq = 5000), Pin.cpu.A8, Pin.cpu.A9)

pitch = 4.0     # [mm]
ema = 0.3
oversample = 4 

steer_kp = 0.12
steer_ki = 0
steer_kd = 0.003
sensor_pins = [Pin.cpu.C3, Pin.cpu.A4, Pin.cpu.C2, Pin.cpu.A1, Pin.cpu.C1, Pin.cpu.A0, Pin.cpu.B0, Pin.cpu.C5, Pin.cpu.A6, Pin.cpu.B1, Pin.cpu.A7, Pin.cpu.C4]

left_PID = Controller(kp=10000,ki=400,kd=0)
right_PID = Controller(kp=10000,ki=400,kd=0)

line_sensor = LineSensor(sensor_pins, pitch=pitch, oversample=oversample, ema=ema)
steer_PID = Controller(kp=steer_kp, ki=steer_ki, kd=steer_kd, setpoint=0.0,u_min=-0.0015, u_max=0.0015)

# # calibrated = False
# def calibrate_line_sensor():
#     global calibrated
#     if calibrated:
#          return                 # Skip if already calibrated
print("Calibrating line sensor... hold over WHITE surface")
pyb.delay(5000)             # holds for 5 seconds
line_sensor.calibrate_white()

print("Now hold over BLACK surface")
pyb.delay(5000)
line_sensor.calibrate_black()
print("Calibration complete.")

# calibrated = True

def compute_steered_setpoints(s_new_setpoint_L, s_new_setpoint_R):          # Read sensor to steering PID and return cmds
     pos_mm, strength = line_sensor.sense_line()
     delta = steer_PID.update(pos_mm)
     base = 0.5*(s_new_setpoint_L.get() + s_new_setpoint_R.get())
     return (base - delta, base + delta)



def run_L(shares):  
        s_mot_cmd, s_mot_eff_L,s_pos_L, s_vel_L, s_time_L, s_new_setpoint_L, s_new_setpoint_R, s_bump_mask = shares
        state_L = 1
        while True:
            if state_L == 1:                  # Motor off
                mot_left.set_effort(0)
                mot_left.disable()
                if s_mot_cmd.get() == 1.0:
                    # calibrate_line_sensor()
                    state_L = 2
                yield 0
                
            elif state_L == 2:                # Motor on and set effort
                mot_left.enable()          # keep driver awake
                enc_left.zero()

                mask = int(s_bump_mask.get())
                if mask != 0:
                    mot_left.set_effort(0)         # or mot_right in run_R
                    mot_left.disable()
                    s_mot_cmd.put(0.0)             # tell everyone we’re stopped
                    state_L = 1                    # back to idle (use your state var)
                    yield 0
                    continue

                left_PID.last_error = 0.0
                left_PID.integral_error = 0.0
                left_PID.last_time = None
                left_PID.setpoint = float(s_new_setpoint_L.get())

                steer_PID.last_error = 0.0
                steer_PID.integral_error = 0.0
                steer_PID.last_time = None
                steer_PID.setpoint = 0.0

                # effort_L = s_mot_eff_L.get()
                # mot_left.set_effort(effort_L)
                sp_L = s_new_setpoint_L.get()
                left_PID.set_setpoint(sp_L)
                steer_PID.reset(sp_L)

                stepstart_L = pyb.micros()
                while s_mot_cmd.get() == 1.0:
                    vL_cmd, vR_cmd = compute_steered_setpoints(s_new_setpoint_L, s_new_setpoint_R)
                    s_new_setpoint_L.put(vL_cmd)
                    s_new_setpoint_R.put(vR_cmd)
                    
                    enc_left.update()
                    pos_L = enc_left.get_position()
                    vel_L = enc_left.get_velocity()
                    tL = pyb.micros()
                    s_pos_L.put(pos_L)
                    s_vel_L.put(vel_L)
                    s_time_L.put(tL)       

                    left_PID.setpoint = float(s_new_setpoint_L.get())
                    u_L = left_PID.update(vel_L)
                    s_mot_eff_L.put(u_L)
                    mot_left.set_effort(u_L)

                    elapsed_L = pyb.elapsed_micros(stepstart_L)
                    s_pos_L.put(pos_L)
                    s_vel_L.put(vel_L)
                    s_time_L.put(elapsed_L)

                    if s_bump_mask.get() != 0:
                        mot_left.set_effort(0)
                        mot_left.disable()
                        s_mot_cmd.put(0.0)
                        yield 0
                        continue

                    yield 0
                
                # s_mot_cmd.put(0.0)
                mot_left.set_effort(0)
                mot_left.disable()
                state_L = 1
                yield 0
                # for i in range(11):
                #     effort = i*10
                #     s_mot_eff_L.put(effort)
                #     mot_left.set_effort(effort)
                #     enc_left.zero()
                #     stepstart_L = pyb.millis()
                #     for j in range(50):                   # waits for 1000 scheduler loops - change value to change wait time
                #         enc_left.update()
                #         pos_L = enc_left.get_position()
                #         vel_L = enc_left.get_velocity()                # publish so DataCollect/plots can see motion
                #         elapsed_L = pyb.elapsed_millis(stepstart_L)
                #         s_pos_L.put(pos_L)
                #         s_vel_L.put(vel_L)
                #         s_time_L.put(elapsed_L)
                #         yield 0
                #     mot_left.set_effort(0)
                #     for j in range(50):                   # waits for 1000 scheduler loops - change value to change wait time
                #         yield 0

  
                # mot_left.set_effort(0)      # ADDED these three instead because for closed loop control, we want each one to be controlled separately       
                # mot_left.disable()
                # state_L = 1          

def run_R(shares):  
        s_mot_cmd, s_mot_eff_R, s_pos_R, s_vel_R, s_time_R, s_new_setpoint_R, s_new_setpoint_L, s_bump_mask = shares
        state_R = 1
        while True:
            if state_R == 1:                  # Motor off
                mot_right.set_effort(0)
                mot_right.disable()
                if s_mot_cmd.get() == 1.0:
                    state_R = 2
                yield 0

            elif state_R == 2:                # Motor on and set effort
                mot_right.enable()          # keep driver awake
                enc_right.zero()

                mask = int(s_bump_mask.get())
                if mask != 0:
                    mot_right.set_effort(0)         # or mot_right in run_R
                    mot_right.disable()
                    s_mot_cmd.put(0.0)             # tell everyone we’re stopped
                    state_R = 1                    # back to idle (use your state var)
                    yield 0
                    continue

                right_PID.last_error = 0.0
                right_PID.integral_error = 0.0
                right_PID.last_time = None
                right_PID.setpoint = float(s_new_setpoint_R.get())
                
                steer_PID.last_error = 0.0
                steer_PID.integral_error = 0.0
                steer_PID.last_time = None
                steer_PID.setpoint = 0.0

                # effort_R = s_mot_eff_R.get()
                # mot_right.set_effort(effort_R)
                sp_R = s_new_setpoint_R.get()
                right_PID.set_setpoint(sp_R)
                steer_PID.reset(sp_R)

                stepstart_R = pyb.micros()
                while s_mot_cmd.get() == 1.0:
                    vL_cmd, vR_cmd = compute_steered_setpoints(s_new_setpoint_L, s_new_setpoint_R)
                    s_new_setpoint_L.put(vL_cmd)
                    s_new_setpoint_R.put(vR_cmd) 

                    enc_right.update()
                    pos_R = enc_right.get_position()
                    vel_R = enc_right.get_velocity()
                    tR = pyb.micros()
                    s_pos_R.put(pos_R)
                    s_vel_R.put(vel_R)
                    s_time_R.put(tR)

                    right_PID.setpoint = float(s_new_setpoint_R.get())
                    u_R = right_PID.update(vel_R)
                    s_mot_eff_R.put(u_R)
                    mot_right.set_effort(u_R)

                    elapsed_R = pyb.elapsed_micros(stepstart_R)
                    s_pos_R.put(pos_R)
                    s_vel_R.put(vel_R)
                    s_time_R.put(elapsed_R)

                    if s_bump_mask.get() != 0:
                        mot_right.set_effort(0)
                        mot_right.disable()
                        s_mot_cmd.put(0.0)
                        yield 0
                        continue

                    yield 0
               
                mot_right.set_effort(0)
                mot_right.disable()
                state_R = 1  
 

# def line_follow(shares):
#     s_mot_cmd, s_new_setpoint_L, s_new_setpoint_R = shares
     
#     print("Calibrating line sensor... hold over WHITE surface")
#     pyb.delay(5000)             # holds for 5 seconds
#     line_sensor.calibrate_white()

#     print("Now hold over BLACK surface")
#     pyb.delay(5000)
#     line_sensor.calibrate_black()
#     print("Calibration complete.")

#     # while True:
#     #     pos, strength = ls.sense_line()
#     #     print("pos={:.2f} mm, strength={:.0f}".format(pos, strength))
#     #     pyb.delay(100)

#     state = 0
#     while True:
#         if state == 0:
#             if s_mot_cmd.get() == 1.0:
#                 steer_PID.set_setpoint(0.0)
#                 state = 1
#             yield 0 

#         elif state == 1:
#             pos_mm, strength = line_sensor.sense_line()
#             delta = steer_PID.update(pos_mm)
#             base = 0.5*(s_new_setpoint_L.get() + s_new_setpoint_R())
#             s_new_setpoint_L.put(base-delta)
#             s_new_setpoint_R.put(base+delta)
#             if s_mot_cmd.get() == 0.0:
#                 state = 0
#             yield 0
              