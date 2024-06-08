import machine
from math import sin, cos, tan, atan, atan2, pi, sqrt, asin, acos
import time
import json
import sys
import mpu6050
import fusion


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
	
	def set_raw(self, duty_us):
		self.duty_ns = int(duty_us * 1000)
		if self.pwm is not None:
			self.pwm.duty_ns(self.duty_ns)

class ServoSafetyControllerProxy:
	def __init__(self, servo_safety_controller, index):
		self.servo_safety_controller = servo_safety_controller
		self.index = index
	
	def set_angle(self, angle):
		self.servo_safety_controller.set_angle(self.index, angle)

	def force_enable(self):
		self.servo_safety_controller.force_enable(self.index)

class ServoSafetyController:
	def __init__(self, servos):
		self.servos = servos
		self.next_servo_powerup_time = time.ticks_ms()
		self.n_servos_powered = 0
		self.proxies = [ServoSafetyControllerProxy(self, i) for i in range(len(servos))]
	
	def set_angle(self, servo, angle):
		self.servos[servo].set_angle(angle)

	def force_enable(self, index):
		self.servos[index].enable()
	
	def loop(self):
		now = time.ticks_ms()
		time_until_next = time.ticks_diff(self.next_servo_powerup_time, now)

		if self.n_servos_powered < len(self.servos) and time_until_next <= 0:
			self.servos[self.n_servos_powered].enable()
			self.n_servos_powered += 1
			self.next_servo_powerup_time = time.ticks_add(now, 100)

def servo_calib_one_value(servo, initial):
	step = 64
	value = initial
	print("adjust rotation: 1-9 for step width, + or -, end with space")
	while True:
		servo.set_raw(value)
		print(value)

		cmd = sys.stdin.read(1)
		if cmd >= '1' and cmd <= '9':
			step = 2 ** (ord(cmd) - ord('1'))
		if cmd == '+' or cmd == '=':
			value += step
		if cmd == '-':
			value -= step
		if cmd == ' ':
			break

	return value

def servo_calib(servo):
	servo.enable()
	print("adjust center position")
	center = servo_calib_one_value(servo, servo.center)
	print("adjust any 90 deg position, or skip by pressing enter")
	rot90 = servo_calib_one_value(servo, center)
	if abs(rot90 - center) <= 100: # implausibly low, was probably skipped
		print("skipped")
	else:
		servo.offset90 = abs(center-rot90)
	servo.center = center


class CurrentMeasuring:
	def __init__(self, mux_a_pin, mux_b_pin):
		self.a = mux_a_pin
		self.b = mux_b_pin
		self.next_index = 0
		self.update_mux(self.next_index)

		self.adcs = [machine.ADC(i) for i in [0,1,2]]

		self.values = [0] * 12

		FULL_PERIOD = 20
		self.timer = machine.Timer(mode=machine.Timer.PERIODIC, period = int(FULL_PERIOD/4), callback = lambda t: self.timer_callback(t))

	def timer_callback(self, t):
		for i in range(3):
			self.values[self.next_index*3 + i] = self.adcs[i].read_u16()

		self.next_index = (self.next_index + 1) % 4
		self.update_mux(self.next_index)

	def update_mux(self, i):
		self.a.value(i & 1 != 0)
		self.b.value(i & 2 != 0)



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

	def force_power_on(self, x, y, z):
		"""only for debugging, normally, servos should be powered on via SafetyController"""
		self.pos(x, y, z)
		self.shoulder.force_enable()
		time.sleep(0.5)
		self.upper.force_enable()
		time.sleep(0.5)
		self.knee.force_enable()


servos = [Servo(i, 1500, 1000) for i in range(12)]

try:
	with open("calib.json", 'r') as file:
		calib = json.load(file)

		for s, c in zip(servos, calib):
			s.center = c['center']
			s.offset90 = c['offset90']

		print("loaded %d calibs" % len(calib))
except:
	print("failed to calibrate servos, using defaults")

print("Calibrate servos? y/n")
if sys.stdin.read(1) == 'y':
	for servo in servos:
		servo_calib(servo)
	
	calib = [ {'center': s.center, 'offset90': s.offset90} for s in servos ]
	with open("calib.json", 'w') as file:
		json.dump(calib, file)
	print("saved")

ctrl = ServoSafetyController(servos)
s = ctrl.proxies

leg_fl = Leg(s[9], s[10], s[11], 1, -1, 1, FRONT)
leg_fr = Leg(s[6], s[7], s[8], 1, 1, -1, FRONT)
leg_rl = Leg(s[5], s[4], s[3], -1, -1, 1, REAR)
leg_rr = Leg(s[2], s[1], s[0], -1, 1, -1, REAR)

legs = [leg_rr, leg_rl, leg_fr, leg_fl]

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
	h = -115
	step_h = 40
	step_len = 60

	step_dur = 500
	for t in range(0,10000, dt_ms):
		
		tt = (t / step_dur) % 4.0
		leg = int(tt)
		phase = tt % 1.0

		phase2 = clamp((phase - 0.5) * 1.5 + 0.5, 0, 1)

		xs = [-step_len/2] * 4
		zs = [0] * 4

		for i in range(0, leg):
			xs[i] += step_len

		xs[leg] = -step_len/2 * cos(phase2 * pi)
		zs[leg] = step_h * sin(phase2 * pi)

		for i in range(4):
			#xs[i] -= tt / 4 * step_len
			xs[i] += 0.5*step_len
		#xs[0] += -37/2
		#xs[3] += 37/2

		xmid = sum(xs)/4
		zmid = sum(zs)/4
		for i in range(4):
			xs[i] -= xmid
			#zs[i] -= zmid

		lean_targets = [(-1, 1), (1, -1), (-1, -1), (1, 1)]
		lean_x_factor = 20 * sin(phase * pi)
		lean_y_factor = 20 * sin(phase * pi)

		lean_x = lean_targets[leg][0] * lean_x_factor
		lean_y = lean_targets[leg][1] * lean_y_factor

		for l, x, z in zip([leg_fr, leg_rl, leg_fl, leg_rr], xs, zs):
			l.pos(x - lean_x, 0 - lean_y, z + h)

		ctrl.loop()
		time.sleep(dt_ms / 1000)
print("hello")

current_measuring = CurrentMeasuring(machine.Pin(19, mode=machine.Pin.OUT), machine.Pin(18, mode=machine.Pin.OUT))

def level(ampl=0, freq=1/3):
	offsets = [0]*4
	CYCLE_TIME=0.01
	phase = 0
	while True:
		phase += freq * CYCLE_TIME * 2 * 3.141592654
		phase %= (2*3.141592654)
		target = 100 + ampl*sin(phase)
		time.sleep(CYCLE_TIME)
		currents = [ sum(current_measuring.values[3*i:3*i+3]) for i in range(4) ]

		avg_current = sum(currents)/4
		current_diffs = [c - avg_current for c in currents]

		offsets = [o- CYCLE_TIME * 0.03 * d for o,d in zip(offsets, current_diffs)]
		offsets = [clamp(o, -40, 40) for o in offsets]
		avg_offset = sum(offsets)/4
		offsets = [o-avg_offset for o in offsets]
		offsets = [clamp(o, -40, 40) for o in offsets]

		#offsets = [o * (0.00001 ** CYCLE_TIME) for o in offsets]

		print(currents, offsets)

		for leg, off in zip(legs, offsets):
			leg.pos(0,0,target+off)
		
		ctrl.loop()


class MotionTracker:
	def __init__(self, imu):
		self.imu = imu
		self.fusion = fusion.Fusion()
		self.timer = machine.Timer(mode=machine.Timer.PERIODIC, period = 10, callback = lambda t: self.timer_callback(t))
	
	def timer_callback(self, timer):
		values = self.imu.get()
		self.fusion.update_nomag(
			[values['accel']['x'], values['accel']['y'], values['accel']['z']],
			[values['gyro']['x'], values['gyro']['y'], values['gyro']['z']]
		)
	
	def get(self):
		return self.fusion.pitch, self.fusion.roll

i2c = machine.I2C(0, scl=machine.Pin(21), sda=machine.Pin(20), freq=400000)
imu = mpu6050.MPU6050(i2c)
motion_tracker = MotionTracker(imu)



#twerk(dt_ms=25)
#get_up(dt_ms=25)

#walk()
