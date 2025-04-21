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
	def __init__(self, alpha = 0.9):
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

lean_x = LowpassFilter()
lean_y = LowpassFilter()
height = LowpassFilter()

tilt_x = LowpassFilter()
tilt_y = LowpassFilter()

leg_x = LowpassFilter()
leg_z = LowpassFilter()

base_x = 0
base_y = 20
base_z = 120

sleep_x = 15
sleep_y = 0
sleep_z = 65

selected_leg = None

tilt_or_height = 'height'

buttons_old = [False] * js.get_numbuttons()
sleep = True
sleep_slow = LowpassFilter(0.97)

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

	lean_x.update(js.get_axis(1))
	lean_y.update(-js.get_axis(0))

	tilt_x.update(js.get_axis(2))
	if tilt_or_height == 'height':
		height.update(js.get_axis(3))
		tilt_y.update(0)
	else:
		height.update(0)
		tilt_y.update(js.get_axis(3))

	# leg is lifted
	leg_z_raw = (js.get_axis(4) + 1) / 2
	leg_x_raw = (js.get_axis(5) + 1) / 2
	print(leg_z_raw)
	leg_z.update(leg_z_raw)
	if leg_z.get() >= 0.02:
		if selected_leg is None:
			side = (lean_y.get() > 0, lean_x.get() > 0)
			if side == (True, True): selected_leg = LEG_FR
			elif side == (False, True): selected_leg = LEG_FL
			elif side == (True, False): selected_leg = LEG_RR
			elif side == (False, False): selected_leg = LEG_RL
			
		leg_x.update(leg_x_raw)
	else:
		selected_leg = None

	axis6 = -1 if (buttons[13], buttons[14]) == (True, False) else 1 if (buttons[13], buttons[14]) == (False, True) else 0
	base_y += axis6 *0.5
	#base_x += js.get_axis(6) * 0.1
	print("base_y = ",base_y)

	xs = [base_x * leg_sign_x(i) + 70*lean_x.get() for i in range(4)]
	ys = [base_y * leg_sign_y(i) + 70*lean_y.get() for i in range(4)]
	zs = [base_z - 30*height.get() for i in range(4)]

	if selected_leg is not None:
		xs[selected_leg] += 30*leg_x.get()
		zs[selected_leg] -= 30*leg_z.get()

	for i in range(4):
		s = sleep_slow.get()
		xs[i] = (1-s) * xs[i] + s* leg_sign_x(i)* sleep_x
		ys[i] = (1-s) * ys[i] + s* leg_sign_y(i)* sleep_y
		zs[i] = (1-s) * zs[i] + s* sleep_z

	for i, name in [(LEG_FL, 'fl'), (LEG_FR, 'fr'), (LEG_RL, 'rl'), (LEG_RR, 'rr')]:
		print("Leg %s: %-6.3f  %-6.3f  %-6.3f" % (f"[{name}]" if i == selected_leg else f" {name} " , xs[i], ys[i], zs[i]))

	if ser is not None:
		line = ""
		for i, name in [(LEG_FL, 'fl'), (LEG_FR, 'fr'), (LEG_RL, 'rl'), (LEG_RR, 'rr')]:
			line += f"leg_{name}.pos({xs[i]}, {ys[i]}, {zs[i]}); "
		line += '\r\n'
		ser.write(line.encode('ascii'))
		ser.read(9999)

	print()
	print()
