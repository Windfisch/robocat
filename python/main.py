import machine
from math import sin, cos, tan, atan, atan2, pi, sqrt, asin, acos, exp
import time
import json
import sys
import mpu6050
import fusion
import stdinreader
from filters import HighPassFilter, LowPassFilter
from pid_controller import PidController


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

class ConstantFrameTime:
	def __init__(self, frame_time_ms):
		self._frame_time_ms = frame_time_ms
		self._last_frame_start_ms = time.ticks_ms()
	
	def wait(self):
		now = time.ticks_ms()
		wait_ms = max(0, self._last_frame_start_ms + self._frame_time_ms - now)
		if wait_ms <= 0:
			print("Frame time violation! Frame took %4.2fms but was supposed to take %4.2fms" % (now - self._last_frame_start_ms, self._frame_time_ms))
		time.sleep(wait_ms / 1000)
		self._last_frame_start_ms = now

def servo_calib_one_value(servo, initial):
	step = 64
	value = initial
	print("adjust rotation: 1-9 for step width, + or -, end with space")
	for i in range(3):
		servo.set_raw(value+50)
		time.sleep(.1)
		servo.set_raw(value)
		time.sleep(.1)

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

	# FIXME: this is wrong. it assumes that the upper shoulder joint rotates along a point right above the paw.
	# but it doesn't, it's inset by 1-2cm
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
leg_sign_x = [-1, -1, +1, +1]
leg_sign_y = [+1, -1, +1, -1]

LEG_RR = 0
LEG_RL = 1
LEG_FR = 2
LEG_FL = 3

def add3d(a, b):
	return (a[i] + b[i] for i in range(3))

#def matmul(m, v):
#	return (m[


# Coordinate systems:
# "world coordinate system": kinda irrelevant because we don't know our absolute x,y position.
# "gravity coordinate system": (0,0,0) is in the robots's center (center of the top wooden plate),
#                              z = upwards (parallel to the gravity vector),
#                              x = forward*, y = left (*: parallel to the horizontal plane)
# "robot coordinate system": (0,0,0) is in the robot's center,
#                             z = upwards (orthogonal to the robot's top plane)
#                             x = forward**, y = left** (**: parallel to the robot's top plane)
class Tetrapod:
	def __init__(self, legs, xlen, ylen, default_z):
		self.legs = legs
		self.xlen = xlen
		self.ylen = ylen
		self.default_z = default_z

	# paw positions are given in the gravity coordinate system but shifted so that
	# every leg's preferred position is (0,0,0).
	# pitch and roll are in degrees.
	def pos_rel(self, positions, pitch, roll):
		self.pos_abs(
			self,
			[
				add3d(positions[LEG_RR], (-self.xlen/2, -self.ylen/2, self.default_z)),
				add3d(positions[LEG_RL], (-self.xlen/2,  self.ylen/2, self.default_z)),
				add3d(positions[LEG_FR], ( self.xlen/2, -self.ylen/2, self.default_z)),
				add3d(positions[LEG_FL], ( self.xlen/2,  self.ylen/2, self.default_z)),
			],
			pitch, roll
		)

	# paw positions are given in the gravity coordinate system.
	# pitch and roll are in degrees.
	#def pos_abs(self, positions, pitch, roll):
		

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

	hp_pitch = HighPassFilter(1 / CYCLE_TIME)
	hp_roll = HighPassFilter(1 / CYCLE_TIME)

	while True:
		phase += freq * CYCLE_TIME * 2 * 3.141592654
		phase %= (2*3.141592654)
		target = 100 + ampl*sin(phase)
		time.sleep(CYCLE_TIME)
		#currents = [ sum(current_measuring.values[3*i:3*i+3]) for i in range(4) ]
		currents = [0]*4

		desired_currents = [0]*4

		pitch, roll = motion_tracker.get()
		desired_pitch, desired_roll = 0,0
		pitch_error, roll_error = (pitch - desired_pitch, roll - desired_roll)

		dpitch, droll = hp_pitch.update(pitch), hp_roll.update(roll)

		xpos = 0.7*dpitch
		ypos = 1*droll

		LEVEL_FACTOR = 800
		#LEVEL_FACTOR = 0
		desired_currents[LEG_RR] += LEVEL_FACTOR * (-pitch_error + roll_error)
		desired_currents[LEG_RL] += LEVEL_FACTOR * (-pitch_error - roll_error)
		desired_currents[LEG_FR] += LEVEL_FACTOR * (pitch_error + roll_error)
		desired_currents[LEG_FL] += LEVEL_FACTOR * (pitch_error - roll_error)

		avg_current = sum(currents)/4
		current_diffs = [c - avg_current - desired for (c,desired) in zip(currents, desired_currents)]

		offsets = [o- CYCLE_TIME * 0.03 * d for o,d in zip(offsets, current_diffs)]
		offsets = [clamp(o, -30, 40) for o in offsets]
		avg_offset = sum(offsets)/4
		offsets = [o-avg_offset for o in offsets]
		offsets = [clamp(o, -30, 40) for o in offsets]

		#offsets = [o * (0.00001 ** CYCLE_TIME) for o in offsets]

		print(desired_currents, currents, offsets, dpitch, droll)

		for leg, off in zip(legs, offsets):
			leg.pos(xpos,ypos,target+off)
		
		ctrl.loop()

def step_curve(t):
	MOVE=0.33
	HOLD=0.0
	if t < MOVE: return 0.5 - cos(t/MOVE*pi)/2, 1
	t-=MOVE
	if t < HOLD: return 1, 2
	t-=HOLD
	if t < MOVE: return 0.5 + cos(t/MOVE*pi)/2, 3
	return 0, 0

def w2():
	with open("log.csv", "w") as f:
		walk2(f)

def walk2(file = None):
	r = stdinreader.StdinReader()
	v = {
		'x1': 0,
		'y1': 0,
		'z1': 80,
		'x2': 0,
		'y2': 0,
		'z2': 100,
		'pR': 0,
		'iR': 0.2,
		'ilR': 999,
		'dR': 0,
		'angle': 45,
		'basex': 0,
		'basey': 0,
	}

	R1controller = PidController(0,0,0)
	R2controller = PidController(0,0,0)

	FRAME_TIME = 20 # ms

	frametime = ConstantFrameTime(FRAME_TIME)
	R1highpass = HighPassFilter(20 / FRAME_TIME)
	R2highpass = HighPassFilter(20 / FRAME_TIME)

	diag1 = [LEG_FL, LEG_RR]
	diag2 = [LEG_RL, LEG_FR]

	# ideally, this should be the vector from the RR to the FL leg (normalized)
	angle_x = cos(v['angle'] / 180 * pi)
	angle_y = sin(v['angle'] / 180 * pi)

	if file: file.write("# " + "\t".join(["$%d=%s" % (i, x) for (i, x) in enumerate(['time_ms', 'step', 'step_phase', 'inhibit_i', 'pitch', 'roll', 'tilt', 'error', 'ctrl', 'ctrl_i_accu'])]) + "\n")

	while True:
		ctrl.loop()
		frametime.wait()

		step, step_phase = step_curve((time.ticks_ms()/1000) % 1.0)
		v['z1'] = 100 - 20*step

		inhibit_i = 0 if step_phase in {1, 2} else 1


		line = r.getline()
		if line:
			for (variable, value) in [stmt.split('=') for stmt in line.replace(' ','').split(';')]:
				if variable in v:
					try:
						v[variable] = float(value)
					except ValueError:
						pass
			
			# ideally, this should be the vector from the RR to the FL leg (normalized)
			angle_x = cos(v['angle'] / 180 * pi)
			angle_y = sin(v['angle'] / 180 * pi)

		
		R1controller.p = v['pR']
		R1controller.i = v['iR']
		R1controller.i_limit = v['ilR']
		R1controller.d = v['dR']
		R2controller.p = v['pR']
		R2controller.i = v['iR']
		R2controller.i_limit = v['ilR']
		R2controller.d = v['dR']

		pitch, roll = motion_tracker.get()
		#print(pitch, roll)
		pitch = pitch / 180 * pi
		roll = roll / 180 * pi

		if abs(v['z1'] - v['z2']) < 0.1:
			# standing on all four legs
			# update no controller
			error = 0
			tilt = 0
			R1highpass.reset()
			R2highpass.reset()
		elif v['z1'] < v['z2']:
			# diag1's legs are lifted, diag2 is standing on the ground
			axis_x, axis_y = angle_x, angle_y

			# now we'll take (axis_x, axis_y, 0)^T in the robot frame ("XYZ") and rotate it into the
			# world frame ("xyz") by multiplying:  R_z(0) * R_y(pitch) * R_x(roll) * (axis_x, axis_y, 0)^T

			# ( cos p      sin r sin p   cos r sin p )   (  axis_x  )
			# (  0         cos r        -sin r       ) * (  axis_y  )
			# ( -sin p     sin r cos p   cos p cos r )   (  0       )

			# and compute the scalar product with (0,0,1)^T
			# this is cos( 90deg + tilt ), where tilt is the tilt angle against the horizontal (rotating across the standing two legs, ideally)
			# positive tilt means that the axis is pointing downwards, i.e. that our center of gravity is too far in the direction of the axis
			# and needs to move away
			tilt = acos( -sin(pitch)*axis_x + sin(roll)*cos(pitch)*axis_y ) * 180 / pi  -  90
			
			#print('tilt', tilt)
			error = R1highpass.update(tilt)
			R2controller.update(error, inhibit_i)
			print(tilt, error, inhibit_i)
		elif v['z1'] > v['z2']:
			# diag2's legs are lifted, diag1 is standing on the ground
			axis_x, axis_y = -angle_x, angle_y
			tilt = acos( -sin(pitch)*axis_x + sin(roll)*cos(pitch)*axis_y ) * 180 / pi  -  90
			error = R2highpass.update(tilt)
			R1controller.update(error, inhibit_i)

		if file: file.write(f"{time.ticks_ms()}\t{step}\t{step_phase}\t{inhibit_i}\t{pitch*180/pi}\t{roll*180/pi}\t{tilt*180/pi}\t{error*180/pi}\t{R2controller.value()}\t{R2controller.i_accumulator}\n")

		#print(R1controller.value(), R2controller.value())
		for i in diag1:
			axis_x, axis_y = -angle_x, angle_y
			x = v['basex'] * leg_sign_x[i] + v['x1'] + axis_x * R1controller.value()
			y = v['basey'] * leg_sign_y[i] + v['y1'] - axis_y * R1controller.value()
			z = v['z1']
			#print("leg #%d:  %5.2f  %5.2f  %5.2f" % (i,x,y,z))
			legs[i].pos(x,y,z)
		
		for i in diag2:
			axis_x, axis_y = angle_x, angle_y
			x = v['basex'] * leg_sign_x[i] + v['x2'] + axis_x * R2controller.value()
			y = v['basey'] * leg_sign_y[i] + v['y2'] - axis_y * R2controller.value()
			z = v['z2']
			#print("leg #%d:  %5.2f  %5.2f  %5.2f" % (i,x,y,z))
			legs[i].pos(x,y,z)
		


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
