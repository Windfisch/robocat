from math import sin, cos, tan, atan, atan2, pi, sqrt, asin, acos

import sys

def pos(self, x, y, z):
	j1,j2,j3 = 28, 50, 66

	# what shall the shoulder angle be?
	shoulder = atan(-y / z)

	print(shoulder)

	# now we think from the upper joint's coordinate system:
	# what x and z coordinates does the leg need to have?
	# (y1 is always zero, because both upper and knee
	# joint rotate in the XZ plane, leaving Y unchanged)
	x1 = x
	z1 = j1 - sqrt(y**2 + z**2)
	upper_offset = atan(x1 / z1)

	print(x1, z1, upper_offset)

	# and now we compute the intersection point I of a
	# circle with radius j2 around (0,0) and a
	# circle with radius j3 around (0,-d)
	d = -sqrt(x1**2 + z1**2)
	#xi = sqrt(-d**4 + 2 * d**2 * (j2**2 + j3**2) - (j2**2 - j3**2)**2) / (2*d)
	zi = (d**2 + j2**2 - j3**2) / (2*d)

	print("d,zi", d, zi)
	upper2 = acos(-zi / j2)
	knee = -acos((zi-d) / j3) - upper2

	upper = upper_offset + upper2

	shoulder = shoulder * 180 / pi
	upper = upper * 180 / pi
	knee = knee * 180 / pi

	print(shoulder, upper, knee)

pos(None, float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))
