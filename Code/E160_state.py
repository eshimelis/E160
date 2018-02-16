import util

class E160_state:

    def __init__(self, x=0, y=0, theta=0):
        self.x = x
        self.y = y
        self.theta = theta

        # self.set_state(0,0,0)
        
    def set_state(self,x,y,theta):
        self.x = x
        self.y = y
        self.theta = theta

    def __add__(self, other):
        return E160_state(self.x + other.x, self.y + other.y, self.theta + other.theta)

    def __sub__(self, other):
        return E160_state(self.x - other.x, self.y - other.y, self.theta - other.theta)

    # deep copy of robot state
    def copy(self):
        return E160_state(self.x, self.y, self.theta)