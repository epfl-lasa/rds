import math

class Capsule:
	def __init__(self, y_front, y_back, r):
		self.y_front = y_front
		self.y_back = y_back
		self.r = r
	
	def distance(self, x, y):
		if y > self.y_front:
			return math.sqrt(x*x + (y - self.y_front)*(y - self.y_front)) - self.r
		elif y < self.y_back:
			return math.sqrt(x*x + (y - self.y_back)*(y - self.y_back)) - self.r
		else:
			return math.fabs(x) - self.r
