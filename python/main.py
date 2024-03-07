import machine
from math import sin, cos, tan, atan, atan2, pi, sqrt, asin, acos
import time

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
		self.pin = machine.PWM(pin, freq = 200, duty_ns = center * 1000)
	
	def set_angle(self, angle):
		self.pin.duty_ns(int((self.center + angle / 90 * self.offset90) * 1000))

class Leg:
	def __init__(self, shoulder, upper, knee, shoulder_sign, upper_sign, knee_sign, side):
		self.shoulder = Servo(shoulder, 1500, shoulder_sign * 1000)
		self.upper = Servo(upper, 1500, upper_sign * 1000)
		self.knee = Servo(knee, 1500, knee_sign * 1000)
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

		self.shoulder.set_angle(shoulder)
		self.upper.set_angle(upper)
		self.knee.set_angle(knee)




leg_fl = Leg(11, 9, 10, 1, -1, 1, FRONT)
leg_fr = Leg(5, 3, 4, 1, 1, -1, FRONT)
leg_rl = Leg(8, 7, 6, -1, -1, 1, REAR)
leg_rr = Leg(2, 0, 1, -1, 1, -1, REAR)

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
		time.sleep(dt_ms / 1000)

print("hello")

twerk(dt_ms=25)
