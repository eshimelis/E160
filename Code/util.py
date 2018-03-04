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