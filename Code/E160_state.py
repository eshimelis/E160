import util
import math
import numpy as np

class E160_state:

    def __init__(self, x=0, y=0, theta=0):
        self.x = x
        self.y = y
        self.theta = theta
        self.DIM = 3    # state space dimension

    def set_state(self,x,y,theta):
        self.x = x
        self.y = y
        self.theta = theta

    def __add__(self, other):
        return E160_state(self.x + other.x, self.y + other.y, util.angle_wrap(self.theta + other.theta))

    def __sub__(self, other):
        return E160_state(self.x - other.x, self.y - other.y, util.angle_wrap(self.theta - other.theta))

    def xydist(self, other):
        return math.sqrt((self.x-other.x)**2 + (self.y-other.y)**2)

    def __repr__(self):
        return "[" + str(self.x) + ", " + str(self.y) + ", " + str(self.theta) + "]"
    # deep copy of robot state
    def copy(self):
        return E160_state(self.x, self.y, self.theta)

    def toVec(self):
        array = np.array([[self.x], [self.y], [self.theta]])
        return array