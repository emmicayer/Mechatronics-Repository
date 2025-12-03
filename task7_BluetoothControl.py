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
                    self.buf.clear()                    # clear buffer
                    if line:
                        return line                     # and return line
            else:
                if len(self.buf) < self.maxlen:         # if length of buffer is less than max
                    self.buf.append(ch)                 # then append to the buffer
                else:                                   # otherwise
                    self.buf.clear()                    # clear the buffer
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

    state = 0

    bt = None
    read = None
    mode = "MANUAL"
    telemetry = False
    cmd = None
    parts = None

    telem_period_ms = 100
    next_telem_ms = pyb.millis() + telem_period_ms

    while True:
        if s_bump_mask != 0:
            s_mot_cmd.put(0.0)

        if state == 0:
            if bt is None:
                bt = UART(1, 115200)
                read = LineReader(bt)

            try:
                bt.write(b"BT READY\r\n")
            except:
                pass
            state = 1
            yield 0

        elif state == 1:
            line = read.readline()
            if line is None:
                now = pyb.millis()
                if telemetry and now >= next_telem_ms:
                    state = 4
                else:
                    state = 1
                yield 0
                continue

            parts = _tok(line)
            state = 2
            yield 0

        elif state == 2:
            if not parts:
                state = 1
                yield 0
                continue

            cmd = parts[0].upper()
            state = 3
            yield 0

        elif state == 3:
            try:
                if cmd == "PING":
                    bt.write(b"PONG\r\n")

                elif cmd == "GO":
                    s_mot_cmd.put(1.0)
                    bt.write(b"GO\r\n")

                elif cmd == "STOP":
                    s_mot_cmd.put(0.0)
                    bt.write(b"STOP\r\n")

                elif cmd == "MODE" and len(parts) >= 2:
                    m = parts[1].upper()
                    if m in ("MANUAL", "LINE"):
                        mode = m
                        bt.write(b"MODE %s\r\n" % m.encode())
                    else:
                        bt.write(b"ERROR MODE\r\n")

                elif cmd == "VEL":
                    l_val, r_val = None, None
                    for p in parts[1:]:
                        pU = p.upper()
                        if pU.startswith("L="):
                            l_val = parse_float(p[2:], None)
                        elif pU.startswith("R="):
                            r_val = parse_float(p[2:], None)
                    if l_val is not None:
                        s_new_setpoint_L.put(float(l_val))
                    if r_val is not None:
                        s_new_setpoint_R.put(float(r_val))
                    bt.write(b"VEL L=%.6f R=%.6f\r\n" % (s_new_setpoint_L.get(), s_new_setpoint_R.get()))

                elif cmd == "TELEM" and len(parts) >= 2:
                    if parts[1].upper() == "ON":
                        telemetry = True
                        bt.write(b"TELEM ON\r\n")
                    elif parts[1].upper() == "OFF":
                        telemetry = False
                        bt.write(b"TELEM OFF\r\n")
                    else:
                        bt.write(b"ERROR TELEM\r\n")

                elif cmd == "GET" and len(parts) >= 2 and parts[1].upper() == "STATUS":
                    bump = int(s_bump_mask.get())
                    bt.write(("STATUS mode={} bump=0x{:X} mot_cmd={:.1f}\r\n".format(mode, bump, s_mot_cmd.get())).encode())

                else:
                    bt.write(b"ERROR UNKNOWN\r\n")
            except:
                pass
            state = 1
            yield 0

        elif state == 4:
            try:
                bump = int(s_bump_mask.get())
                bt.write(("T mode{} L=%.6f R=%.6f bump=0x{:X} cmd={:.1f}\r\n".format(mode) % (s_new_setpoint_L.get(), s_new_setpoint_R.get(),bump, s_mot_cmd.get())).encode())
            except:
                pass
            next_telem_ms = pyb.millis() + telem_period_ms
            state = 1
            yield 0