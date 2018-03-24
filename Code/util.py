import math

def angle_wrap(a):
    while a > math.pi:
        a = a - 2*math.pi
    while a < -math.pi:
        a = a + 2*math.pi
        
    return a

def ang_diff(a, b):
	d = angle_wrap(a - b)
	# d = (d + math.pi)%(2*math.pi) - math.pi
	return d

def ang_sum(a, b):
	d = angle_wrap(a + b)
	# d = (d + math.pi)%(2*math.pi) - math.pi
	return d


class Vec2:
	def __init__(self, x=0.0, y=0.0):
		self.x = float(x)
		self.y = float(y)

	def slope_to(self, point2):
		return float(point2.y - self.y)/float(point2.x - self.x)

	def cross(self, other):
		return self.x*other.y - self.y*other.x

	def scale(self, scaleFactor):
		return Vec2(self.x * scaleFactor, self.y * scaleFactor)
	
	def copy(self):
		return Vec2(self.x, self.y)

	def __add__(self, other):
		return Vec2(self.x + other.x, self.y + other.y)

	def __sub__(self, other):
		return Vec2(self.x - other.x, self.y - other.y)

	def norm(self):
		return math.sqrt(self.x**2 + self.y**2)

	def __repr__(self):
		return "[" + str(self.x) + ", " + str(self.y) + "]"
