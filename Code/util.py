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


class Point:

	def __init__(self, x=0, y=0):
		self.x = 0
		self.y = 0


	def slope_to(self, point2):
		return float(point2.y - self.y)/float(point2.x - self.x)