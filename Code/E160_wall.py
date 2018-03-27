class E160_wall:
    
    def __init__(self, wall_points):
        
        # set up walls

        if (wall_points[2] - wall_points[0] == 0):
            self.slope = float('inf')
        else: 
            self.slope = float(wall_points[3] - wall_points[1])/(wall_points[2] - wall_points[0])

        self.wall_points = wall_points