## @file task7_BluetoothControl.py
#  @brief Bluetooth command and telemetry task for controlling the robot via a
#  UART connection and receiving real-time status updates. This replaces the user interface in User_Input
#  when Bluetooth is in use. 
#
#  This task processes incoming text-based commands sent over Bluetooth and
#  updates shared variables accordingly. Supported functions include:
#     • Starting and stopping the robot (GO / STOP)
#     • Setting left and right velocity setpoints (VEL L=… R=…)
#     • Changing control modes (MANUAL / LINE)
#     • Enabling or disabling telemetry streaming
#     • Returning robot status (GET STATUS)
#     • Basic connection testing (PING, which returns PONG)
#
#  Commands are parsed using a lightweight line reader with buffer management.
#  When telemetry is enabled, the task periodically transmits motor setpoints,
#  mode, bump-mask status, and motor command state.
#
#  The task runs as a non-blocking state machine, yielding frequently to maintain
#  responsiveness alongside other system tasks.

import pyb
from pyb import UART

class LineReader:
    def __init__(self, uart, bufsize=128):
        self.uart = uart
        self.buf = bytearray()                          # create a buffer
        self.maxlen = bufsize

    def readline(self):
        while self.uart.any():                          # if any character is pressed,
            b = self.uart.read(1)                       # read 1 character
            if not b:
                break
            ch = b[0]
            if ch in (10,13):                           # If character is newline, then
                if self.buf:
                    line = bytes(self.buf).strip()      # strip whitespace,
                    self.buf[:]=b''                    # clear buffer
                    if line:
                        return line                     # and return line
            else:
                if len(self.buf) < self.maxlen:         # if length of buffer is less than max
                    self.buf.append(ch)                 # then append to the buffer
                else:                                   # otherwise
                    self.buf[:]=b''                    # clear the buffer
        return None                                     # if there is nothing to be read, return None

def _tok(line):
    parts = line.decode().strip().split()               # decodes, strips, and splits line
    return parts

def parse_float(val, default=None):
    try:
        return float(val)                               # turn value into a float
    except:
        return default

def BluetoothTask(shares):
    s_mot_cmd, s_new_setpoint_L, s_new_setpoint_R, s_bump_mask = shares

    state = 1

    bt = None
    read = None
    mode = "MANUAL"
    telemetry = False
    cmd = None
    parts = None

    telem_period_ms = 100
    next_telem_ms = pyb.millis() + telem_period_ms

    help_txt = (b"\r\nCommands:\r\n"
            b"  PING: test BT and get PONG\r\n"
            b"  GO: Run Romi\r\n"
            b"  STOP: Stop Romi\r\n"
            b"  VEL L=? R=?: Set motor speed\r\n"
            b"  GET STATUS: Get status report of bump sensor and motor command\r\n"
            b"\r\n")

    while True:
        if s_bump_mask.get() != 0:
            s_mot_cmd.put(0)

        if state == 1:
            if bt is None:
                bt = UART(1, 115200)
                read = LineReader(bt)

            try:
                bt.write(b"BT READY\r\n")
                bt.write(help_txt)
            except:
                pass
            state = 2
            yield 0

        elif state == 2:
            line = read.readline()
            if line is None:
                now = pyb.millis()
                yield 0
                continue

            parts = _tok(line)
            state = 3
            yield 0

        elif state == 3:
            if not parts:
                state = 2
                yield 0
                continue

            cmd = parts[0].upper()
            state = 4
            yield 0

        elif state == 4:
            try:
                if cmd == "PING":
                    bt.write(b"PONG\r\n")

                elif cmd == "GO":
                    s_mot_cmd.put(1)
                    bt.write(b"GO\r\n")

                elif cmd == "STOP":
                    s_mot_cmd.put(0)
                    bt.write(b"STOP\r\n")

                elif cmd == "VEL":
                    l_val, r_val = None, None
                    table = {'0': 0, '1': 0.00032, '2': 0.00064, '3': 0.00096, '4': 0.00128, '5': 0.0016,
                             '6': 0.00192, '7': 0.00224, '8': 0.00256, '9': 0.00288, '10': 0.0032}
                    for p in parts[1:]:
                        pU = p.upper()
                        if pU.startswith("L="):
                            char = p[2:].strip()
                            if char in table:
                                l_val = float(table[char])
                            else:
                                bt.write(b"ERROR\r\n")
                        elif pU.startswith("R="):
                            char = p[2:].strip()
                            if char in table:
                                r_val = float(table[char])
                            else:
                                bt.write(b"ERROR\r\n")
                    if l_val is not None:
                        s_new_setpoint_L.put(float(l_val))
                    if r_val is not None:
                        s_new_setpoint_R.put(float(r_val))
                    bt.write(b"VEL L=%.6f R=%.6f\r\n" % (s_new_setpoint_L.get(), s_new_setpoint_R.get()))

                elif cmd == "GET" and len(parts) >= 2 and parts[1].upper() == "STATUS":
                    bump = int(s_bump_mask.get())
                    bt.write(("STATUS bump=0x{:X} mot_cmd={:.1f}\r\n".format( bump, s_mot_cmd.get())).encode())

                else:
                    bt.write(b"ERROR UNKNOWN\r\n")
            except:
                pass
            state = 2
            yield 0