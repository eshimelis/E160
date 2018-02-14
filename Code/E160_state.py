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