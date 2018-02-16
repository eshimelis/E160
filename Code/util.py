import math

# def angDiff(theta1, theta2):




# bound angles in 
def boundAngleRad(theta):    
    while(theta>math.pi):
        theta -= 2.0*math.pi
    while(theta<math.pi):
        theta += 2.0*math.pi
    return theta

def boundAngleDeg(theta):
    while(theta>180):
        theta -= 360
    while(theta<180):
        theta += 360
    return theta