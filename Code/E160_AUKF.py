import math
import random
import numpy as np
import copy
import util
from E160_state import *

# from scipy.stats import norm
import scipy as sp

class E160_AUKF:

    def __init__(self, environment, spaceDim, robotWidth, wheelRadius, encoderResolution, initialState = None, covariance = None):
        self.sigmaPoints = []
        self.environment = environment

        # ukf parameters
        self.n = spaceDim   # dimension of state space
        self.wDim = 2
        self.vDim = 3

        self.alpha = 0.001   # papers recommends 1e-3 (spread of sigma points)
        self.beta = 2       #
        self.kappa = 0      # usually set to 0 (secondary scaling parameter)
        self.lam = (self.alpha**2) * (self.n + self.kappa) - self.n

        delta_s_std = 0.01
        delta_theta_std = 0.1

        num_sensors = 3
        sensor_std = 0.1

        W_t = [ [delta_s_std**2, 0],
                [0, delta_theta_std**2]]
        
        self.W_t = np.array(W_t)    
        self.V_t = (sensor_std**2) * np.identity(num_sensors)

        if covariance == None:
            # self.cov = 0.01
            # self.covariance = self.cov*np.identity(self.n)

            cov = [ [0.01, 0.0001, 0.0001],
                    [0.0001, 0.01, 0.0001],
                    [0.0001, 0.0001, 0.01]]
            self.covariance = np.array(cov)
        else:
            self.covariance = covariance

        self.augDim = self.n + self.wDim + self.vDim
        self.aug_covariance = np.zeros((self.augDim, self.augDim))

        # add state covariance
        offset = 0
        self.aug_covariance[offset:self.covariance.shape[0]+offset,offset:self.covariance.shape[1]+offset] = self.covariance

        # add process covariance
        offset = 3
        self.aug_covariance[offset:self.W_t.shape[0]+offset,offset:self.W_t.shape[1]+offset] = self.W_t

        # add measurement covariance
        offset = 5
        self.aug_covariance[offset:self.V_t.shape[0]+offset,offset:self.V_t.shape[1]+offset] = self.V_t

        # print("augmented matrix")
        # print(self.aug_covariance)

        self.numSigPoints = 2*self.augDim + 1

        # maybe should just pass in a robot class?
        self.robotWidth = robotWidth
        self.radius = robotWidth/2
        self.wheelRadius = wheelRadius
        self.encoderResolution = encoderResolution
        self.FAR_READING = 3

        self.DIM = spaceDim

        # for particle weight normalization
        self.particle_weight_sum = 0

        self.start = True

        # define the sensor orientations
        self.sensor_orientation = [0, -math.pi/2, math.pi/2] # orientations of the sensors on robot
        self.walls = self.environment.walls

        # initialize the current state0
        if initialState == None:
            self.state = E160_state()
            self.state.set_state(0,0,0)

            self.state_prev = E160_state()
            self.state_prev.set_state(0,0,0)
        else: 
            self.state = initialState.copy()
            self.state_prev = initialState.copy()

        self.map_maxX = 4.0
        self.map_minX = 0
        self.map_maxY = 4.0
        self.map_minY = 0

        # self.last_encoder_measurements = [0,0]

    def GenerateSigmaPoints(self, mean, covariance):
        ''' Initialize a set of sigma points about the mean '''
        
        # intialize sigma set
        X = np.zeros((self.augDim, self.numSigPoints))
        
        self.meanWeights = []
        self.covWeights = []

        # add mean and weights to sigma sets
        self.meanWeights.append(self.lam/(self.n + self.lam))
        self.covWeights.append(self.meanWeights[0] + (1-self.alpha**2 + self.beta))
        
        X[:, [0]] = mean

        # ensure matrix symmetry
        covariance = (1/2) * (covariance + np.transpose(covariance))

        # compute square root of covariance matrix
        # covariance = np.absolute(covariance)
        rootP = sp.linalg.sqrtm(covariance)
        # rootP = covariance
        # rootP = np.linalg.cholesky(covariance)
        
        gamma = math.sqrt(self.augDim + self.lam)

        # print(rootP)

        #### Verify or Remove ####
        # rootP = np.real(rootP)
        #### Verify or Remove ####

        # intialize remaining sigma points
        meanWeight = 1/(2*(self.n + self.lam))
        covWeight = meanWeight

        print(X.shape)

        for i in range(1, self.augDim):
            print(i)
            # create and append both positive and negative sigma points
            sigmaPointA = mean + gamma*rootP[:, [i]]
                        
            self.meanWeights.append(meanWeight)
            self.covWeights.append(covWeight)
            
            X[:, [i]] = sigmaPointA
            
        for i in range(0, self.augDim):
            sigmaPointB = mean - gamma*rootP[:, [i]]
            
            self.meanWeights.append(meanWeight)
            self.covWeights.append(covWeight)
            
            X[:, [i+self.augDim]] = sigmaPointB

        print(X)
        return X

    def Propagate(self, Xprev, delta_s, delta_theta):
        '''Propagate all the particles from the last state with odometry readings
            Args:
                delta_s (float): distance traveled based on odometry
                delta_theta(float): change in heading based on odometry
            return:
                nothing'''

        X = []

        for i in range(self.numSigPoints):
            state = self.SigmaPoint(Xprev[0, i], Xprev[1, i], Xprev[2, i])

            delta_x = delta_s * math.cos(state.theta + delta_theta/2)
            delta_y = delta_s * math.sin(state.theta + delta_theta/2)

            # minus because robot is backwards
            X[:, [i]] = Xprev[:, [i]]
            X[0, i] -= delta_x
            X[1, i] -= delta_y
            X[2, i] = util.angle_wrap(X[2, i] + delta_theta)

        return X

    def LocalizeEstWithUKF(self, delta_s, delta_theta, sensor_readings):
        ''' Localize the robot with Unscented Kalman filters. Call everything
            Args: 
                delta_s (float): change in distance as calculated by odometry
                delta_heading (float): change in heading as calcualted by odometry
                sensor_readings([float, float, float]): sensor readings from range fingers
            Return:
                None'''

        # create augmented state vector
        zFront = sensor_readings[0]
        zRight = sensor_readings[1]
        zLeft = sensor_readings[2]
        self.aug_state = np.transpose(np.array([[self.state.x, self.state.y, self.state.theta, delta_s, delta_theta, zFront, zRight, zLeft]]))

        print("Augmented State:\n", self.aug_state, "\n")

        ## [2] generate set of sigma points
        Xprev = self.GenerateSigmaPoints(self.aug_state, self.aug_covariance)

        #------------------------------------------------------------------
        ## [3] propagate set of sigma points
        Xbar_star = self.Propagate(Xprev, delta_s, delta_theta) 

        #------------------------------------------------------------------
        ## [4] calculate mu bar
        muBarVec = np.zeros((self.n, 1))
        for i in range(self.numSigPoints):
            muBarVec += Xbar_star[i].mWeight * Xbar_star[i].toVec()

        #------------------------------------------------------------------
        ## [5] calculate sigma bar
        sigmaBar = np.zeros((self.n, self.n))
        for i in range(self.numSigPoints):
            sigmaBar += Xprev[i].cWeight * np.matmul((Xbar_star[i].toVec() - muBarVec), np.transpose(Xbar_star[i].toVec() - muBarVec))
        sigmaBar += self.R_t # add additive noise

        #------------------------------------------------------------------
        ## [6] new sigma points
        # convert mubarVec to a sigma point
        muBar = self.SigmaPoint(muBarVec[0,0], muBarVec[1,0], muBarVec[2,0], Xprev[0].mWeight, Xprev[0].cWeight)
        Xbar = self.GenerateSigmaPoints(muBar, sigmaBar)

        #------------------------------------------------------------------
        ## [7] compute expected measurements (Z_t)
        Zbar = self.ComputeExpSensor(Xbar)

        #------------------------------------------------------------------
        ## [8] compute average measurement (z_t)
        zBarVec = np.zeros((self.n, 1))
        for i in range(self.numSigPoints):
            zBarVec += Xprev[i].mWeight * Zbar[:,[i]]

        #------------------------------------------------------------------
        ## [9] compute expected uncertainty (S_t)
        s_t = np.zeros((len(self.sensor_orientation), len(self.sensor_orientation)))
        for i in range(self.numSigPoints):
            s_t += Xprev[i].cWeight * np.matmul((Zbar[:,[i]] - zBarVec), np.transpose(Zbar[:,[i]] - zBarVec))
        s_t += self.Q_t # add additive sensor noise

        #------------------------------------------------------------------
        ## [10] compute cross coveriance Sigma bar_x,z
        sigmaBar_XZ = np.zeros((self.n, len(self.sensor_orientation)))
        for i in range(self.numSigPoints):
            # sigmaBar_XZ += Xprev[i].cWeight * np.matmul((Xbar[i].toVec() - muBarVec), np.transpose(Zbar[:,[i]] - zBarVec))
            sigmaBar_XZ += Xprev[i].cWeight * np.matmul((Xprev[i].toVec() - muBarVec), np.transpose(Zbar[:,[i]] - zBarVec))

        #------------------------------------------------------------------
        ## [11] compute Kalman Gain (K_t)
        # K_t = np.zeros((self.n, self.n))
        s_tInv = np.linalg.inv(s_t)
        K_t = np.matmul(sigmaBar_XZ, s_tInv)

        print("\n\nKalman Gain:\n", K_t)

        #------------------------------------------------------------------
        ## [12] update state estimation (mu)
        # convert sensor readings to numpy array 
        sensor_var = 0.3
        front_noise = np.random.normal(0, sensor_var)
        right_noise = np.random.normal(0, sensor_var)
        left_noise = np.random.normal(0, sensor_var)

        z = np.array([[sensor_readings[0]+front_noise], [sensor_readings[1]+right_noise], [sensor_readings[2]+left_noise]])
        mu = muBarVec + np.matmul(K_t, (z - zBarVec))

        #------------------------------------------------------------------
        ## [13] update covariance estimation (sigma)
        sigma = sigmaBar - np.matmul(K_t, np.matmul(s_t, np.transpose(K_t)))

        #------------------------------------------------------------------
        
        # update state and covariance and save sigma points
        self.state.set_state(mu[0,0], mu[1,0], mu[2,0])
        self.covariance = sigma
        self.sigmaPoints = Xbar

        print("\n Covariance:\n", self.covariance, "\n")

        return self.state

    def ComputeExpSensor(self, X):

        Z = np.zeros((len(self.sensor_orientation), self.numSigPoints))
        
        # compute measurement vector for each sigma point
        for i in range(self.numSigPoints):
            sp = X[i]
            z = np.zeros((len(self.sensor_orientation), 1))

            # find expected wall distance for each sensor orientation
            for j in range(len(self.sensor_orientation)):
                z[j,0] = self.FindMinWallDistance(sp, self.walls, self.sensor_orientation[j])
            Z[:, [i]] = z

        return Z


    def FindMinWallDistance(self, sigmaPoint, walls, sensorT):
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
            wallDist = self.FindWallDistance(sigmaPoint, wall.wall_points, sensorT)
            if wallDist == None:
                minDist = min(minDist, self.FAR_READING)
            else: minDist = min(minDist, wallDist)
            
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
        def __init__(self, x, y, theta):
            self.x = x
            self.y = y
            self.theta = theta

            self.n = 3  # dimension of state space

        def __repr__(self):
            return "[" + str(self.x) + ", " + str(self.y) + ", " + str(self.theta) + "]"

        def toVec(self):
            array = np.array([[self.x], [self.y], [self.theta]])
            return array
        
        def set_state(self, x, y, theta):
            self.x = x
            self.y = y
            self.theta = theta

        def set_weight(self, newWeight):
            self.weight = newWeight

        def copy(self):
            return E160_UKF.SigmaPoint(self.x, self.y, self.theta)
