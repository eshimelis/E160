import math
import random
import numpy as np
import copy
import util
from E160_state import *
from scipy.stats import norm


class E160_PF:

    def __init__(self, environment, robotWidth, wheel_radius, encoder_resolution):
        self.particles = []
        self.environment = environment
        self.numParticles = 200
        
        # maybe should just pass in a robot class?
        self.robotWidth = robotWidth
        self.radius = robotWidth/2
        self.wheel_radius = wheel_radius
        self.encoder_resolution = encoder_resolution
        self.FAR_READING = 10
        
        # PF parameters
        self.IR_sigma = 0.1 # Range finder s.d
        # self.odom_xy_sigma = 1.25   # odometry delta_s s.d
        # self.odom_heading_sigma = 0.75  # odometry heading s.d
        self.odom_xy_sigma = 0.05  # odometry standard deviation, meters
        self.odom_heading_sigma = 0.03  # odometry heading s.d
        # self.odom_xy_sigma = 0.3  # odometry standard deviation, meters
        # self.odom_heading_sigma = 0.3 # odometry heading s.d

        self.delta_rl_sigma = 0.05

        self.particle_weight_sum = 0

        # define the sensor orientations
        self.sensor_orientation = [0, math.pi/2, -math.pi/2] # orientations of the sensors on robot
        self.walls = self.environment.walls

        # initialize the current state
        self.state = E160_state()
        self.state.set_state(0,0,0)

        # TODO: change this later
        self.map_maxX = 1.0
        self.map_minX = -1.0
        self.map_maxY = 1.0
        self.map_minY = -1.0
        self.InitializeParticles()
        self.last_encoder_measurements =[0,0]

    def InitializeParticles(self):
        ''' Populate self.particles with random Particle 
            Args:
                None
            Return:
                None'''
        self.particles = []
        for i in range(0, self.numParticles):
            self.SetRandomStartPos(i)
            # startPos = E160_state(0, 0, 0)
            # self.SetKnownStartPos(i, startPos)

            
    def SetRandomStartPos(self, i):
        xPos = random.uniform(self.map_minX, self.map_maxX)
        yPos = random.uniform(self.map_minY, self.map_maxY)
        theta = random.uniform(-math.pi, math.pi)
        self.particles.append(self.Particle(xPos,yPos,theta,1))

    def SetKnownStartPos(self, i):
        self.particles.append(self.Particle(0,0,0,1))
            
    def LocalizeEstWithParticleFilterEncoder(self, encoder_measurements, sensor_readings):
        ''' Localize the robot with particle filters. Call everything
            Args: 
                econder_measurements([float, float]): encoder reading measurements
                sensor_readings([float, float, float]): sensor readings from range fingers
            Return:
                None'''
        
        # for i in range(self.numParticles):

        #     delta_s = 0
        #     delta_theta = 0

        #     encoder_delta_left = encoder_measurements[0] - self.last_encoder_measurements[0]
        #     encoder_delta_right = encoder_measurements[1] - self.last_encoder_measurements[1]

        #     delta_s_left = float(encoder_delta_left)*(math.pi*self.wheel_radius*2)/self.encoder_resolution
        #     delta_s_right = float(encoder_delta_right)*(math.pi*self.wheel_radius*2)/self.encoder_resolution

        #     # delta_s_left += np.random.normal(0, self.delta_rl_sigma)
        #     # delta_s_right += np.random.normal(0, self.delta_rl_sigma)

        #     delta_s = (delta_s_left + delta_s_right)/2
        #     delta_theta = (delta_s_left - delta_s_right)/(2*self.radius)

        #     self.Propagate(delta_s, delta_theta, i)
        #     self.particles[i].set_weight(self.CalculateWeight(sensor_readings, self.environment.walls, self.particles[i]))

        # self.Resample()
        # return self.GetEstimatedPos()

    def LocalizeEstWithParticleFilter(self, state_odo, delta_s, delta_theta, sensor_readings):
        ''' Localize the robot with particle filters. Call everything
            Args: 
                delta_s (float): change in distance as calculated by odometry
                delta_heading (float): change in heading as calcualted by odometry
                sensor_readings([float, float, float]): sensor readings from range fingers
            Return:
                None'''
        for i in range(self.numParticles):
            # propogate and weight each particle based on sensor reading
            self.Propagate(delta_s, delta_theta, i)
            self.particles[i].set_weight(self.CalculateWeight(sensor_readings, self.environment.walls, self.particles[i]))

        self.Resample()
        return self.GetEstimatedPos()

    def Propagate(self, delta_s, delta_theta, i):
        '''Propagate all the particles from the last state with odometry readings
            Args:
                delta_s (float): distance traveled based on odometry
                delta_theta(float): change in heading based on odometry
            return:
                nothing'''
        # add student code here 

        
        # if abs(delta_s) > 0 or abs(delta_theta) > 0:
        delta_s += np.random.normal(0, self.odom_xy_sigma)
        delta_theta += np.random.normal(0, self.odom_heading_sigma)
            
        state = self.particles[i]

        delta_x = delta_s * math.cos(state.theta + delta_theta/2)
        delta_y = delta_s * math.sin(state.theta + delta_theta/2)


        # minus because robot is backwards
        self.particles[i].set_state(state.x - delta_x, state.y - delta_y, util.angle_wrap(state.theta + delta_theta))
        
        # end student code here
        
        
    def CalculateWeight(self, sensor_readings, walls, particle):
        '''Calculate the weight of a particular particle
            Args:
                particle (E160_Particle): a given particle
                sensor_readings ( [float, ...] ): readings from the IR sesnors
                walls ([ [four doubles], ...] ): positions of the walls from environment, 
                            represented as 4 doubles 
            return:
                new weight of the particle (float) '''

        newWeight = 1
        # for reading in sensor_readings:
        #         
        # convert measurement to standard normal
        for wall in walls:

            for i in range(len(self.sensor_orientation)):
                expDist = min(self.FindMinWallDistance(particle, walls, self.sensor_orientation[i]), self.FAR_READING)
                actDist = min(sensor_readings[i], self.FAR_READING)

                normalizedMeasurement = (expDist - actDist)/self.IR_sigma
                newWeight *= norm.pdf(normalizedMeasurement)
        # print(newWeight)
        return newWeight


    def Resample(self):
        '''Resample the particles systematically
            Args:
                None
            Return:
                None'''
        
        self.particle_weight_sum = 0
        for i in range(self.numParticles):
            self.particle_weight_sum += self.particles[i].weight
        
        newParticles = []

        for i in range(self.numParticles):
            r = random.uniform(0, self.particle_weight_sum)
            j = 0
            wsum = self.particles[j].weight
            while(wsum < r):
                j += 1
                wsum += self.particles[j].weight

            particle = self.particles[j]
            newParticles.append(self.Particle(particle.x, particle.y, particle.theta, 1))

        self.particles = newParticles

    def GetEstimatedPos(self):
        ''' Calculate the mean of the particles and return it 
            Args:
                None
            Return:
                None'''
        
        xSum = 0
        ySum = 0
        thetaSum = 0

        for i in range(self.numParticles):
            xSum += self.particles[i].x
            ySum += self.particles[i].y
            thetaSum += self.particles[i].theta

        self.state.set_state(xSum/self.numParticles, ySum/self.numParticles, thetaSum/self.numParticles)
        # print("Estimated State: ", self.state)
        return self.state


    def FindMinWallDistance(self, particle, walls, sensorT):
        ''' Given a particle position, walls, and a sensor, find 
            shortest distance to the wall
            Args:
                particle (E160_Particle): a particle 
                walls ([E160_wall, ...]): represents endpoint of the wall 
                sensorT: orientation of the sensor on the robot
            Return:
                distance to the closest wall' (float)'''
        
        minDist = float('inf')
        
        for wall in walls:
            wallDist = self.FindWallDistance(particle, wall.wall_points, sensorT)
            if wallDist != None:
                minDist = min(minDist, wallDist)

        # print(minDist)
        return minDist
    

    def FindWallDistance(self, particle, wall, sensorT):
        ''' Given a particle position, a wall, and a sensor, find distance to the wall
            Args:
                particle (E160_Particle): a particle 
                wall ([float x4]): represents endpoint of the wall 
                sensorT: orientation of the sensor on the robot
            Return:
                distance to the closest wall (float)'''
        
        point1 = util.Vec2(wall[0], wall[1])
        point2 = util.Vec2(wall[2], wall[3])
        point2 = point2 - point1 # direction vector from point 1

        bot = util.Vec2(particle.x, particle.y) # bot vector
        heading = util.Vec2(math.cos(util.angle_wrap(particle.theta+sensorT)), math.sin(util.angle_wrap(particle.theta+sensorT))) # heading vector

        # intersection exists if
        # bot + v*heading = point1 + u*point2Dir

        # check if lines are parallel
        if heading.cross(point2) == 0:
            return None
        else:
            v = (point1 - bot).cross(point2) / heading.cross(point2)    # bot vector scaling factor
            u = (bot - point1).cross(heading) / point2.cross(heading)   # wall vector scaling factor

            # check if intersection exists
            # v > 0 - Robot facing wall
            # 0 < u < 1 - Intersection in wall
            if v < 0 or u < 0 or u > 1:
                return None
            else:
                return heading.scale(v).norm()

    def angleDiff(self, ang):
        ''' Wrap angles between -pi and pi'''
        while ang < -math.pi:
            ang = ang + 2 * math.pi
        while ang > math.pi:
            ang = ang - 2 * math.pi
        return ang

    class Particle(E160_state):
        def __init__(self, x, y, theta, weight):
            self.x = x
            self.y = y
            self.theta = theta
            self.weight = weight

        def __str__(self):
            return str(self.x) + " " + str(self.y) + " " + str(self.theta) + " " + str(self.weight)
        
        def set_weight(self, newWeight):
            if newWeight != None:
                self.weight = newWeight



