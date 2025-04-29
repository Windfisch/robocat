from math import *

class LowPassFilter:
	def __init__(self, time_constant):
		self.time_constant = time_constant
		self.accu = None
	
	def reset(self):
		self.accu = None

	def update(self, value):
		if self.accu is None:
			self.accu = value

		self.accu = self.accu * exp(-1/self.time_constant) + value * (1 - exp(-1/self.time_constant))

		return self.accu
	
	def value(self):
		return self.accu

class HighPassFilter:
	def __init__(self, time_constant):
		self.lp = LowPassFilter(time_constant)
	
	def reset(self):
		self.lp.reset()
	
	def update(self, value):
		self._value = value - self.lp.update(value)
		return self._value
	
	def value(self):
		return self._value

