import math
import random
import numpy as np
import copy
import util
from E160_state import *
# from scipy.stats import norm
import scipy as sp


class E160_UKF:

    def __init__(self, environment, spaceDim, robotWidth, wheelRadius, encoderResolution, initialState = None, covariance = None):
        self.sigmaPoints = []
        self.environment = environment
        
        # ukf parameters
        self.L = spaceDim   # dimension of state space
        self.alpha = 0.1    # papers recommends 1e-3 (spread of sigma points)
        self.beta = 2       #  
        self.kappa = 0      # usually set to 0 (secondary scaling parameter)
        self.lam = (self.alpha**2) * (self.L + self.kappa) - self.L

        if covariance == None: 
            self.covariance = np.array([[0.4, 0, 0], [0, 0.4, 0], [0, 0, 1]])
        else:
            self.covariance = covariance

        self.numSigPoints = 2*self.L + 1
        # maybe should just pass in a robot class?
        self.robotWidth = robotWidth
        self.radius = robotWidth/2
        self.wheelRadius = wheelRadius
        self.encoderResolution = encoderResolution
        self.FAR_READING = 3

        self.DIM = spaceDim
        
        # PF parameters
        if environment.robot_mode == "SIMULATION MODE":
            self.IR_sigma = 0.1 # Range finder s.d
            self.odom_sigma = 0.2
            self.sampling_threshold = 0.6
            self.odom_xy_sigma = 0.03  # odometry standard deviation, meters
            self.odom_heading_sigma = 0.03 # odometry heading s.d
            
        else:
            self.IR_sigma = 0.3 # Range finder s.d
            self.odom_sigma = 0.2
            self.sampling_threshold = 0.5
            self.odom_xy_sigma = 0.03  # odometry standard deviation, meters
            self.odom_heading_sigma = 0.03 # odometry heading s.d

        # for particle weight normalization
        self.particle_weight_sum = 0

        self.start = True

        # define the sensor orientations
        self.sensor_orientation = [0, -math.pi/2, math.pi/2] # orientations of the sensors on robot
        self.walls = self.environment.walls

        # initialize the current state
        if initialState == None: 
            self.state = E160_state()
            self.state.set_state(0,0,0)

            self.state_prev = E160_state()
            self.state_prev.set_state(0,0,0)
        else: 
            self.state = initialState
            self.state_prev = initialState

        # TODO: change this later
        self.map_maxX = 4.0
        self.map_minX = 0
        self.map_maxY = 4.0
        self.map_minY = 0

        # self.last_encoder_measurements = [0,0]

    def GenerateSigmaPoints(self, mean, covariance):
        ''' Initialize a set of sigma points about the mean '''

        self.sigmaPoints = []

        # add mean to set
        meanWeight = self.lam/(self.L + self.lam)
        covWeight = meanWeight + (1-self.alpha**2 + self.beta)
        self.sigmaPoints.append(self.SigmaPoint(mean.x, mean.y, mean.theta, meanWeight, covWeight))

        # compute square root of covariance matrix
        rootP = sp.linalg.sqrtm(covariance)
        scale = math.sqrt(self.L + self.lam)
        
        # print(rootP)
        # print(np.matmul(rootP,rootP))

        # create mean vector
        xVec = mean.toVec()

        # intialize remaining sigma points

        meanWeight = 1/(2*(self.L + self.lam))
        covWeight = meanWeight

        for i in range(self.DIM):
        
            sigmaPointA = xVec + scale*rootP[:, i]
            
            sigmaPointB = xVec - scale*rootP[:, i]

            self.sigmaPoints.append(self.SigmaPoint(sigmaPointA[0], sigmaPointA[1], sigmaPointA[2], meanWeight, covWeight))
            self.sigmaPoints.append(self.SigmaPoint(sigmaPointB[0], sigmaPointB[1], sigmaPointB[2], meanWeight, covWeight))

        return self.sigmaPoints

    def Propagate(self, delta_s, delta_theta, i):
        '''Propagate all the particles from the last state with odometry readings
            Args:
                delta_s (float): distance traveled based on odometry
                delta_theta(float): change in heading based on odometry
            return:
                nothing'''

        state = self.sigmaPoints[i]

        delta_x = delta_s * math.cos(state.theta + delta_theta/2)
        delta_y = delta_s * math.sin(state.theta + delta_theta/2)

        # minus because robot is backwards
        self.sigmaPoints[i].set_state(state.x - delta_x, state.y - delta_y, util.angle_wrap(state.theta + delta_theta))

    def LocalizeEstWithUKF(self, delta_s, delta_theta, sensor_readings):
        ''' Localize the robot with Unscented Kalman filters. Call everything
            Args: 
                delta_s (float): change in distance as calculated by odometry
                delta_heading (float): change in heading as calcualted by odometry
                sensor_readings([float, float, float]): sensor readings from range fingers
            Return:
                None'''

        print(sensor_readings)

        # if self.start == True:
        self.GenerateSigmaPoints(self.state, self.covariance)
            # self.start = False
            # print("hi")

        for i in range(self.numSigPoints):
            self.Propagate(delta_s, delta_theta, i)    

        # keep track of previous state
        self.state_prev = self.state

        self.state = self.UpdateEstimatedPos()
        self.Updatecovariance        
        return self.state
        
    def CalculateWeight(self, sensor_readings, walls, particle):
        '''Calculate the weight of a particular particle
            Args:
                particle (E160_Particle): a given particle
                sensor_readings ( [float, ...] ): readings from the IR sesnors
                walls ([ [four doubles], ...] ): positions of the walls from environment, 
                            represented as 4 doubles 
            return:
                new weight of the particle (float) '''

        weight = particle.weight
        # weight = 1.0

        for i in range(len(self.sensor_orientation)):

            z = min(self.FindMinWallDistance(particle, walls, self.sensor_orientation[i]), self.FAR_READING)
            zexp = min(sensor_readings[i], self.FAR_READING)

            # eta = 0.5
            eta = 0.00118
            lam = 0.1

            # measurement noise model
            normalizedMeasurement = (z - zexp)/(self.IR_sigma)
            measurementNoise = eta*sp.stats.norm.pdf(normalizedMeasurement)

            # unexpected noise model
            if z < zexp:
                unexpectedNoise = eta*lam*math.exp(-lam*z)
            else: unexpectedNoise = 0

            # random noise model
            randomNoise = eta/self.FAR_READING

            # max range model
            if zexp >= self.FAR_READING and z >= self.FAR_READING:
                maxNoise = eta
            else:
                maxNoise = 0

            weight *= (measurementNoise + unexpectedNoise + randomNoise + maxNoise)

        return weight

    def UpdateEstimatedPos(self):
        ''' Calculate the mean of the sigma points and return it 
            Args:
                None
            Return:
                None'''
        
        xSum = 0
        ySum = 0
        sinSum = 0
        cosSum = 0

        totalWeight = 0
        for i in range(self.numSigPoints):
            totalWeight += self.sigmaPoints[i].mWeight

        print("Total Weight: ", totalWeight)

        for i in range(self.numSigPoints):
            weight = self.sigmaPoints[i].mWeight
            xSum += self.sigmaPoints[i].x * (weight/totalWeight)
            ySum += self.sigmaPoints[i].y * (weight/totalWeight)
            sinSum += math.sin(self.sigmaPoints[i].theta) * (weight/totalWeight)
            cosSum += math.cos(self.sigmaPoints[i].theta) * (weight/totalWeight)

        self.state.set_state(xSum, ySum, math.atan2(sinSum, cosSum))
        return self.state

    def Updatecovariance(self):
        ''' Calculate the mean of the sigma points and return it 
            Args:
                None
            Return:
                None'''
       
        for i in range(self.numSigPoints):
            self.covariance *= self.sigmaPoints[i].cWeight

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
            if wallDist == None:
                minDist = min(minDist, self.FAR_READING)
            else: minDist = min(minDist, wallDist)
            
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

    class SigmaPoint(E160_state):
        def __init__(self, x, y, theta, meanWeight, covarianceWeight):
            self.x = x
            self.y = y
            self.theta = theta
            self.mWeight = meanWeight
            self.cWeight = covarianceWeight

        def __repr__(self):
            return "[" + str(self.x) + ", " + str(self.y) + ", " + str(self.theta) + ", " + str(self.weight) + "]"

        def toVec(self):
            return np.transpose(np.array([self.x, self.y, self.theta]))
        
        def set_state(self, x, y, theta):
            self.x = x
            self.y = y
            self.theta = theta

        def set_weight(self, newWeight):
            self.weight = newWeight