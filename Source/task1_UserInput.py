## @file task1_UserInput.py
#  @brief User-input interface task for handling USB serial commands to control
#  the robot’s motor behavior and speed setpoints.
#
#  This file implements a non-blocking state machine that listens for characters
#  over the pyboard’s USB Virtual COM Port (USB_VCP). Through simple keyboard
#  commands, the user can start and stop the robot, adjust left and right motor
#  speed setpoints, run automated test sequences, and toggle debug printing.
#
#  The task consumes and updates shared variables (motor command, motor efforts,
#  left/right speed setpoints) and exposes a menu-driven interface for selecting
#  new velocities using numeric keys. It is designed to run cooperatively inside
#  the scheduler, yielding control frequently while maintaining responsive input
#  handling.
#
#  Supported commands include:
#     g : begin forward motion / automated test
#     s : stop robot
#     e : enter speed-selection menu
#     r : choose right motor setpoint
#     l : choose left motor setpoint
#     0-9, t : select discrete motor speed values
#     d : toggle debug output
#
#  The task ensures safe operation by interpreting commands into structured
#  states, updating shared data, and printing feedback messages over USB to
#  guide the user.


import pyb
from gc import collect

# TASK 1 - user input interaction
def UserInput(shares):
    s_mot_cmd, s_mot_eff_R, s_mot_eff_L, s_new_setpoint_L, s_new_setpoint_R = shares
    vcp = pyb.USB_VCP()  # virtual com port
    state = 1

    DEBUG = True         # <-- toggle this to enable/disable debug printing

    def dbg(msg):
        if DEBUG:
            try:
                vcp.write(("[UI] " + msg + "\r\n").encode())
            except:
                pass

    help_txt = (b"\r\nCommands:\r\n"
            b"  g  : run forward step test\r\n"
            b"  s  : stop forward step test\r\n"
            b"  e  : select motor speed\r\n"
            b"  r  : set right motor speed\r\n"
            b"  l  : set left motor speed\r\n"
            b"  0-9, t  : specify motor speed\r\n"
            b"  d  : toggle debug on/off\r\n"
            b"\r\n")
    vcp.write(help_txt)

    def read_char():
        if vcp.any():
            b = vcp.read(1)
            if b:
                try:
                    ch = b.decode()
                    if DEBUG and ch not in ['\r', '\n']:
                        dbg("read_char() -> '{}'".format(ch))
                    return ch
                except:
                    return 'z'         # to catch errors
        else:
            return 'p'

    while True:
        collect()
        if state == 1:                     # wait for command
            char = read_char()
            if char == 'e':                # if char e is pressed send to state 2 (mot eff set state)
                dbg("state 1 -> 2 (speed menu)")
                state = 2
            elif char == 'g':              # if char g is pressed send to state 3 (go state)
                dbg("Go pressed: s_mot_cmd=1")
                state = 5
            elif char == 's':
                dbg("Stop pressed: s_mot_cmd=0")
                s_mot_cmd.put(0)
                state = 6
            elif char == 'd':
                DEBUG = not DEBUG
                msg = "[UI] DEBUG = {}\r\n".format(DEBUG)
                vcp.write(msg.encode())
            elif char == 'v':
                msg = "[UI] mot_cmd={:.1f}, L_speed={:.1f}, R_speed={:.1f}\r\n".format(
                    s_mot_cmd.get(), s_new_setpoint_L.get(), s_new_setpoint_R.get()
                )
                vcp.write(msg.encode())
            elif char == 'z':              # to catch errors
                vcp.write(b"[UI] read_char failed\r\n")
            yield 0

        elif state == 2:                                # motor setpoint state
            char = read_char()
            if char == 'r':
                dbg("state 2 -> 3 (right speed)")
                state = 3
            elif char == 'l':
                dbg("state 2 -> 4 (left speed)")
                state = 4
            elif char is not None and char != 'p':
                dbg("leaving speed menu -> state 1")
                state = 1
            yield 0

        elif state == 3:                                        # Right motor setpoint state
            char = read_char()
            if char in '0123456789t':
                table = {'0': 0, '1': 0.00032, '2': 0.00064, '3': 0.00096, '4': 0.00128, '5': 0.0016,
                         '6': 0.00192, '7': 0.00224, '8': 0.00256, '9': 0.00288, 't': 0.0032}
                val = float(table[char])
                s_new_setpoint_R.put(val)
                dbg("Set Right speed -> {:.5f}".format(val))
                state = 1
            elif char in ('p', '\r', '\n'):  # no key / newline: stay here
                pass
            elif char == 'x':
                dbg("Right speed canceled")
                state = 1
            yield 0

        elif state == 4:  # Left motor setpoint state
            char = read_char()
            if char in '0123456789t':
                table = {'0': 0, '1': 0.00032, '2': 0.00064, '3': 0.00096, '4': 0.00128, '5': 0.0016,
                         '6': 0.00192, '7': 0.00224, '8': 0.00256, '9': 0.00288, 't': 0.0032}
                val = float(table[char])
                s_new_setpoint_L.put(val)
                dbg("Set Left speed -> {:.5f}".format(val))
                state = 1
            elif char in ('p', '\r', '\n'):  # no key / newline: stay here
                pass
            elif char == 'x':
                dbg("Left speed canceled")
                state = 1
            yield 0

        elif state == 5:                              # automated testing loop through 0.0 to 100.0 speeds  
            s_mot_cmd.put(1)
            dbg("Here I go!")
            state = 1
            yield 0 

        elif state == 6:           
            s_mot_cmd.put(0)                        # 0.0 command indicates motor to stop
            dbg("Greetings from state 6! I should be Stopping!!")
            state = 1
            yield 0