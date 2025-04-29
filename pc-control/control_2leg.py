import pygame
import time
import serial
import sys

if len(sys.argv) >= 2:
	ser = serial.Serial(sys.argv[1], timeout=0)
else:
	print("No serial device given, entering dummy mode")
	ser = None

if ser is not None:
	ser.write(b'\n')

LEG_FL = 0
LEG_FR = 1
LEG_RL = 2
LEG_RR = 3

def leg_sign_y(leg):
	if leg == LEG_FL or leg == LEG_RL: return -1
	else: return 1
def leg_sign_x(leg):
	if leg == LEG_FL or leg == LEG_FR: return 1
	else: return -1

pygame.init()
pygame.joystick.init()

js = pygame.joystick.Joystick(0)

class LowpassFilter:
	def __init__(self, alpha = 0.7):
		self.val = 0
		self.alpha = alpha
		self.first = True
	
	def update(self, value):
		if self.first:
			self.first = False
			self.val = value
		else:
			self.val = self.alpha * self.val + (1-self.alpha) * value
		return self.val
	
	def get(self):
		return self.val

lean_x = [LowpassFilter(), LowpassFilter()]
lean_y = [LowpassFilter(), LowpassFilter()]
height = [LowpassFilter(), LowpassFilter()]


base_x = 0
base_y = 0
base_z = 100

sleep_x = 15
sleep_y = 0
sleep_z = 65

buttons_old = [False] * js.get_numbuttons()
sleep = True
sleep_slow = LowpassFilter(0.97)

LEG_MAP={0:0, 1:1, 2:1, 3:0}

if ser is not None:
	# answer "no" to "calibrate servos?"
	ser.write("n\r\n".encode('ascii'))
	ser.read(9999)

while True:
	time.sleep(1/50)
	pygame.event.pump()

	for i in range(6):
		print(f"Axis #{i}: %-6.3f" % js.get_axis(i))

	print("Buttons: " + "  ".join([f"[{i}]" if js.get_button(i) else f" {i} " for i in range(js.get_numbuttons())]))

	buttons = [js.get_button(i) for i in range(js.get_numbuttons())]
	buttons_justpressed = [buttons[i] and not buttons_old[i] for i in range(js.get_numbuttons())]
	buttons_old = buttons

	if buttons_justpressed[5]:
		sleep = not sleep

	sleep_slow.update(1 if sleep else 0)

	lean_x[0].update(js.get_axis(1))
	lean_y[0].update(-js.get_axis(0))

	lean_x[1].update(js.get_axis(3))
	lean_y[1].update(-js.get_axis(2))
	
	height[0].update(js.get_axis(4)/2+0.5)
	height[1].update(js.get_axis(5)/2+0.5)

	axis6 = -1 if (buttons[13], buttons[14]) == (True, False) else 1 if (buttons[13], buttons[14]) == (False, True) else 0
	base_y += axis6 *0.5
	print("base_y = ",base_y)

	xs = [base_x * leg_sign_x(i) + 10*lean_x[LEG_MAP[i]].get() for i in range(4)]
	ys = [base_y * leg_sign_y(i) + 10*lean_y[LEG_MAP[i]].get() for i in range(4)]
	zs = [base_z - 30*height[LEG_MAP[i]].get() for i in range(4)]

	for i in range(4):
		s = sleep_slow.get()
		xs[i] = (1-s) * xs[i] + s* leg_sign_x(i)* sleep_x
		ys[i] = (1-s) * ys[i] + s* leg_sign_y(i)* sleep_y
		zs[i] = (1-s) * zs[i] + s* sleep_z

	for i, name in [(LEG_FL, 'fl'), (LEG_FR, 'fr'), (LEG_RL, 'rl'), (LEG_RR, 'rr')]:
		print("Leg %s: %-6.3f  %-6.3f  %-6.3f" % (f"[{name}]" if False else f" {name} " , xs[i], ys[i], zs[i]))

	print()
	print()

	if ser is not None:
		line = ""
		for i, name in [(LEG_FL, 'fl'), (LEG_FR, 'fr'), (LEG_RL, 'rl'), (LEG_RR, 'rr')]:
			line += f"leg_{name}.pos({xs[i]}, {ys[i]}, {zs[i]}); "
		line += '\r\nctrl.loop()\r\n'
		ser.write(line.encode('ascii'))
		ser.read(9999)
