# test_line_sensor.py


print("test_line_sensor imported")


from pyb import Pin
from line_sensor import LineSensor
import pyb



print("checkpoint 1")


# list of your actual sensor pins:
pins = [Pin.cpu.C0, Pin.cpu.C1, Pin.cpu.C2, Pin.cpu.C3, Pin.cpu.C4]

ls = LineSensor(pins, pitch=4.0, oversample=4, ema=0.3)

print("Place sensors over WHITE surface...")
pyb.delay(2000)
ls.calibrate_white()

print("Now place sensors over BLACK line...")
pyb.delay(2000)
ls.calibrate_black()

print("Starting live reads... move the robot slowly across the line.")

while True:
    pos, strength = ls.sense_line()
    print("pos={:.2f} mm, strength={:.0f}".format(pos, strength))
    pyb.delay(100)