import pyb
from pyb import Pin

class BumpSensors:
    def __init__(self, pins, debounce_ms=10):
        self.pins = []
        for p in pins:
            pin = p if isinstance(p, Pin) else Pin(p)
            pin.init(mode=Pin.IN, pull=Pin.PULL_UP)
            self.pins.append(pin)
        self.N = len(self.pins)
        self.debounce_ms = int(max(1, debounce_ms))
        self.last_mask = 0
        self.stable_mask = 0
        self.last_change_t = pyb.millis()

    def raw_mask(self):
        m = 0
        for i, pin in enumerate(self.pins):
            val = pin.value()           # 0 = pressed, 1 = released
            if val == 0:
                m |= (1 << i)           # create bitmask for any pressed pins
        return m

    def read(self):
        now = pyb.millis()
        raw = self.raw_mask()
        if raw != self.last_mask:
            self.last_mask = raw
            self.last_change_t = now
            return self.stable_mask, False
        if (now-self.last_change_t) >= self.debounce_ms and raw != self.stable_mask:
            self.stable_mask = raw
            return self.stable_mask, True
        return self.stable_mask, False


def TaskBump(shares):
    s_bump_mask, = shares
    s_bump_mask.put(0)
    bump_pins = [Pin.cpu.A15, Pin.cpu.H0, Pin.cpu.H1, Pin.cpu.A10, Pin.cpu.B3, Pin.cpu.C7]
    bump_sensors = BumpSensors(bump_pins, debounce_ms=10)
    state = 0
    while True:
        if state == 0:
            mask, changed = bump_sensors.read()
            s_bump_mask.put(mask)
            if changed:
                pyb.USB_VCP().write(("bump mask=0x{:X}\r\n".format(mask)).encode())
            for _ in range(2):
                yield 0