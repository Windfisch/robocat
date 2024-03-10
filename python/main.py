import machine
from math import sin, cos, tan, atan, atan2, pi, sqrt, asin, acos
import time
import json

#pins = [machine.PWM(i, freq = 200, duty_ns = 1500_000) for i in range(12)]

FRONT = 0
REAR = 1

RAD2DEG = 180/pi

def clamp(val, lower, upper):
	if val <= lower: return lower
	if val >= upper: return upper
	return val

class Servo:
	def __init__(self, pin, center, offset90):
		self.center = center
		self.offset90 = offset90
		self.pin = pin
		self.pwm = None
		self.duty_ns = center * 1000
	
	def enable(self):
		self.pwm = machine.PWM(self.pin, freq = 200, duty_ns = self.duty_ns)
	
	def is_enabled(self):
		return self.pwm is not None
	
	def set_angle(self, angle):
		self.duty_ns = int((self.center + angle / 90 * self.offset90) * 1000)
		if self.pwm is not None:
			self.pwm.duty_ns(self.duty_ns)

class ServoSafetyControllerProxy:
	def __init__(self, servo_safety_controller, index):
		self.servo_safety_controller = servo_safety_controller
		self.index = index
	
	def set_angle(self, angle):
		self.servo_safety_controller.set_angle(self.index, angle)

class ServoSafetyController:
	def __init__(self, servos):
		self.servos = servos
		self.next_servo_powerup_time = time.ticks_ms()
		self.n_servos_powered = 0
		self.proxies = [ServoSafetyControllerProxy(self, i) for i in range(len(servos))]
	
	def set_angle(self, servo, angle):
		self.servos[servo].set_angle(angle)
	
	def loop(self):
		now = time.ticks_ms()
		time_until_next = time.ticks_diff(self.next_servo_powerup_time, now)

		if self.n_servos_powered < len(self.servos) and time_until_next <= 0:
			self.servos[self.n_servos_powered].enable()
			self.n_servos_powered += 1
			self.next_servo_powerup_time = time.ticks_add(now, 100)

class Leg:
	def __init__(self, shoulder, upper, knee, shoulder_sign, upper_sign, knee_sign, side):
		self.upper_sign = upper_sign
		self.knee_sign = knee_sign
		self.shoulder_sign = shoulder_sign
		self.shoulder = shoulder
		self.upper = upper
		self.knee = knee
		self.side = side

	def pos(self, x, y, z):
		j1,j2,j3 = 28, 50, 66

		# what shall the shoulder angle be?
		shoulder = atan(-y / z)

		#print(shoulder)

		# now we think from the upper joint's coordinate system:
		# what x and z coordinates does the leg need to have?
		# (y1 is always zero, because both upper and knee
		# joint rotate in the XZ plane, leaving Y unchanged)
		x1 = x
		z1 = j1 - sqrt(y**2 + z**2)
		upper_offset = atan(x1 / z1)

		#print(x1, z1, upper_offset)

		# and now we compute the intersection point I of a
		# circle with radius j2 around (0,0) and a
		# circle with radius j3 around (0,-d)
		d = -sqrt(x1**2 + z1**2)
		#xi = sqrt(-d**4 + 2 * d**2 * (j2**2 + j3**2) - (j2**2 - j3**2)**2) / (2*d)
		zi = (d**2 + j2**2 - j3**2) / (2*d)

		#print("d,zi", d, zi)
		upper2 = acos(-zi / j2)
		knee = -acos((zi-d) / j3) - upper2

		if self.side == REAR:
			upper2 *= -1
			knee *= -1

		upper = upper_offset + upper2

		shoulder = shoulder * 180 / pi
		upper = upper * 180 / pi
		knee = knee * 180 / pi

		if self.side == FRONT:
			knee += 90
		else:
			knee -=90
		
		#print(shoulder, upper, knee)
		shoulder = clamp(shoulder, -20, 20)
		upper = clamp(upper, -90, 90)
		knee = clamp(knee, -90, 90)
		#print(shoulder, upper, knee)

		self.shoulder.set_angle(shoulder * self.shoulder_sign)
		self.upper.set_angle(upper * self.upper_sign)
		self.knee.set_angle(knee * self.knee_sign)

ctrl = ServoSafetyController([Servo(i, 1500, 1000) for i in range(12)])
s = ctrl.proxies

leg_fl = Leg(s[11], s[9], s[10], 1, -1, 1, FRONT)
leg_fr = Leg(s[5], s[3], s[4], 1, 1, -1, FRONT)
leg_rl = Leg(s[8], s[7], s[6], -1, -1, 1, REAR)
leg_rr = Leg(s[2], s[0], s[1], -1, 1, -1, REAR)

legs = [leg_fl, leg_fr, leg_rl, leg_rr]

def pos(x,y,z):
	for leg in legs:
		leg.pos(x,y,z)

def twerk(ax = 25, ay = 25, az = 30, dt_ms = 50):
	for t in range(0,15000,dt_ms):
		x = sin(t/1250*2*pi) * ax
		y = cos(t/1250*2*pi) * ay
		z = -120 +az/2 - az/2*cos(t/7500*2*pi)
		pos(x,y,z)
		ctrl.loop()
		time.sleep(dt_ms / 1000)

def get_up(dt_ms = 25):
	for t in range(0,1500, dt_ms):
		hf = 0
		hr = 0
		leg_fl.pos(21,0,-50-hf)
		leg_fr.pos(21,0,-50-hf)
		leg_rl.pos(-21,0,-50-hr)
		leg_rr.pos(-21,0,-50-hr)
		ctrl.loop()
		time.sleep(dt_ms / 1000)
	for t in range(0,1500, dt_ms):
		hf = 0
		hr = t / 1500 * 70
		leg_fl.pos(21,0,-50-hf)
		leg_fr.pos(21,0,-50-hf)
		leg_rl.pos(-21,0,-50-hr)
		leg_rr.pos(-21,0,-50-hr)
		ctrl.loop()
		time.sleep(dt_ms / 1000)
	for t in range(0,1500, dt_ms):
		hf = t / 1500 * 70
		hr = 70
		leg_fl.pos(21,0,-50-hf)
		leg_fr.pos(21,0,-50-hf)
		leg_rl.pos(-21,0,-50-hr)
		leg_rr.pos(-21,0,-50-hr)
		ctrl.loop()
		time.sleep(dt_ms / 1000)
	for t in range(0,1500, dt_ms):
		hf = 70 - t / 1500 * 70
		hr = 70 - t / 1500 * 70
		leg_fl.pos(21,0,-50-hf)
		leg_fr.pos(21,0,-50-hf)
		leg_rl.pos(-21,0,-50-hr)
		leg_rr.pos(-21,0,-50-hr)
		ctrl.loop()
		time.sleep(dt_ms / 1000)

def walk(dt_ms = 25):
	h = -120
	step_h = 30
	step_len = 50

	step_dur = 1500
	for t in range(0,15000, dt_ms):
		
		tt = (t / step_dur) % 4.0
		leg = int(tt)
		phase = tt % 1.0

		xs = [-step_len/2] * 4
		zs = [0] * 4

		for i in range(0, leg):
			xs[i] += step_len

		xs[leg] = -step_len/2 * cos(phase * pi)
		zs[leg] = step_h * sin(phase * pi)

		#xmid = sum(xs)/4
		#zmid = sum(zs)/4
		#for i in range(4):
		#	xs[i] -= xmid
		#	zs[i] -= zmid
		for i in range(4):
			xs[i] -= tt / 4 * step_len

		for l, x, z in zip([leg_fr, leg_rl, leg_fl, leg_rr], xs, zs):
			l.pos(x, 0, z + h)

		ctrl.loop()
		time.sleep(dt_ms / 1000)
print("hello")

#twerk(dt_ms=25)
#get_up(dt_ms=25)

walk()
