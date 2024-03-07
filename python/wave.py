import machine
from math import sin, pi
import time

pins = [machine.PWM(i, freq = 200, duty_ns = 1500_000) for i in range(12)]

def wave(pin, amplitude, freq, n):
	fps = 30
	for t in range(fps*n/freq):
		pin.duty_ns(1500_000 + int(1000 * amplitude * sin(t/fps*freq*pi*2)))
		time.sleep(1/fps)
	pin.duty_ns(1500_000)

for pin in pins:
	wave(pin, 500, 1, 3)
