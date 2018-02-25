import math

def angle_wrap(a):
    while a > math.pi:
        a = a - 2*math.pi
    while a < -math.pi:
        a = a + 2*math.pi
        
    return a