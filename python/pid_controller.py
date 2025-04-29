from filters import HighPassFilter, LowPassFilter
from math import *

class PidController:
	def __init__(self, p, i, i_limit, d=0, d_time_constant=1):
		self.p = p
		self.i = i
		self.i_limit = i_limit
		self.i_accumulator = 0
		self.d = d
		self.highpass = HighPassFilter(d_time_constant)
		self.output = 0

	def update(self, error, inhibit_i = 0):
		self.i_accumulator += self.i * error * (1 - max(0, min(1, inhibit_i)))
		self.i_accumulator = max(self.i_accumulator, -self.i_limit)
		self.i_accumulator = min(self.i_accumulator, self.i_limit)
		self.highpass.update(error)
		self.output = self.p * error + self.i_accumulator + self.d * self.highpass.value()
		return self.output

	def value(self):
		return self.output




def test_pid():
	step_response = [0.3679, 0.1353, 0.0498, 0.0183]
	pid1 = PidController(1, 0.1, 999)
	pid2 = PidController(1, 0.1, 999, 0.3, 1)

	pid1.update(0)
	pid2.update(0)
	assert(pid1.value() == pid2.value())
	assert(pid1.value() == 0)

	for i in range(4):
		pid1.update(1)
		pid2.update(1)
		assert abs(pid2.value() - (0.3*step_response[i] + pid1.value())) <= 0.001
		assert pid1.value() == 1 + 0.1 * (i+1)

def test_ilimit():
	from pytest import approx

	pid = PidController(0, 0.1, 1)
	for i in range(10):
		assert pid.update(1) == approx(0.1*(i+1))
	
	for i in range(10):
		assert pid.update(1) == 1
	
	for i in range(20):
		assert pid.update(-1) == approx(0.9 - 0.1*i)

	for i in range(10):
		assert pid.update(-1) == -1
