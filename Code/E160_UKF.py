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
        self.n = spaceDim   # dimension of state space
        self.alpha = 0.0001   # papers recommends 1e-3 (spread of sigma points)
        self.beta = 2       #
        self.kappa = 0      # usually set to 0 (secondary scaling parameter)
        self.lam = (self.alpha**2) * (self.n + self.kappa) - self.n
        # print("lam:", self.lam)

        self.r_t = 0.0001  # process noise
        self.q_t = 0.8  # measurement noise
        self.R_t = self.r_t * np.identity(self.n)
        self.Q_t = self.q_t * np.identity(self.n)

        if covariance == None:
            # self.cov = 0.01
            # self.covariance = self.cov*np.identity(self.n)

            cov = [ [0.0001, 0.00001, 0.00001],
                    [0.00001, 0.0001, 0.00001],
                    [0.00001, 0.00001, 0.001]]
            self.covariance = np.array(cov)
        else:
            self.covariance = covariance

        self.numSigPoints = 2*self.n + 1

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

        self.debug = False
        # self.last_encoder_measurements = [0,0]

    def GenerateSigmaPoints(self, mean, covariance):
        ''' Initialize a set of sigma points about the mean '''

        # intialize sigma set
        X = []

        # add mean to set
        meanWeight = self.lam/(self.n + self.lam)
        covWeight = meanWeight + (1-self.alpha**2 + self.beta)
        # print("Mean Weight: ", meanWeight)
        # print("Covariance Weight: ", covWeight)
        X.append(self.SigmaPoint(mean.x, mean.y, mean.theta, meanWeight, covWeight))

        # ensure matrix symmetry
        covariance = (1/2)*(covariance + np.transpose(covariance))

        # compute square root of covariance matrix
        # covariance = np.absolute(covariance)
        rootP = sp.linalg.sqrtm(covariance)
        # rootP = covariance
        # rootP = np.linalg.cholesky(covariance)

        gamma = math.sqrt(self.n + self.lam)

        if self.debug:
            print("\nCovariance: \n", covariance)
            print("\nRootP: \n", rootP)

        #### Verify or Remove ####
        # rootP = np.absolute(rootP)
        rootP = np.real(rootP)
        #### Verify or Remove ####

        # create mean vector
        xVec = mean.toVec()

        # intialize remaining sigma points
        meanWeight = 1/(2*(self.n + self.lam))
        covWeight = meanWeight

        # print("Sigma Mean Weight:", meanWeight)
        # print("Sigma Cov Weight: ", covWeight)

        for i in range(self.DIM):
            # create and append both positive and negative sigma points
            # print("XVec", xVec)
            # print("Offset: ", -gamma*rootP[:, [i]], "\n")
            sigmaPointA = self.StateVecDiff(xVec, -gamma*rootP[:, [i]])
            sigmaPointB = self.StateVecDiff(xVec, gamma*rootP[:, [i]])
            spA = self.SigmaPoint(sigmaPointA[0,0], sigmaPointA[1,0], sigmaPointA[2,0], meanWeight, covWeight)
            spB = self.SigmaPoint(sigmaPointB[0,0], sigmaPointB[1,0], sigmaPointB[2,0], meanWeight, covWeight)
            X.append(spA)
            X.append(spB)
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
            state = Xprev[i]

            delta_x = delta_s * math.cos(state.theta + delta_theta/2)
            delta_y = delta_s * math.sin(state.theta + delta_theta/2)

            # minus because robot is backwards
            X.append(self.SigmaPoint(state.x - delta_x, state.y - delta_y, util.angle_wrap(state.theta + delta_theta), state.mWeight, state.cWeight))

        return X

    def LocalizeEstWithUKF(self, delta_s, delta_theta, sensor_readings):
        ''' Localize the robot with Unscented Kalman filters. Call everything
            Args:
                delta_s (float): change in distance as calculated by odometry
                delta_heading (float): change in heading as calcualted by odometry
                sensor_readings([float, float, float]): sensor readings from range fingers
            Return:
                None'''

        ## [2] generate set of sigma points
        Xprev = self.GenerateSigmaPoints(self.state, self.covariance)
        
        if self.debug: 
            print("\nXprev: \n", Xprev)
            input()

        #------------------------------------------------------------------
        ## [3] propagate set of sigma points
        Xbar_star = self.Propagate(Xprev, delta_s, delta_theta) 

        if self.debug: 
            print("\nXbar_star: \n", Xbar_star)
            input()

        #------------------------------------------------------------------
        ## [4] calculate mu bar
        muBarVec = np.zeros((self.n, 1))
        for i in range(self.numSigPoints):
            muBarVec += Xbar_star[i].mWeight * Xbar_star[i].toVec()
        
        muBarVec[2] = util.angle_wrap(muBarVec[2])

        if self.debug: 
            print("\nmuBarVec: \n", muBarVec)
            input()
        #------------------------------------------------------------------
        ## [5] calculate sigma bar
        sigmaBar = np.zeros((self.n, self.n))
        for i in range(self.numSigPoints):
            # sigmaBar += Xprev[i].cWeight * np.matmul(self.StateVecDiff(Xbar_star[i].toVec(), muBarVec), np.transpose(self.StateVecDiff(Xbar_star[i].toVec(), muBarVec)))
            sigmaBar = np.add(Xprev[i].cWeight * np.matmul(self.StateVecDiff(Xbar_star[i].toVec(), muBarVec), np.transpose(self.StateVecDiff(Xbar_star[i].toVec(), muBarVec))), sigmaBar, out = sigmaBar, casting = "unsafe")
        sigmaBar += self.R_t # add additive noise

        if self.debug: 
            print("\nsigmaBar: \n", sigmaBar)
            input()

        #------------------------------------------------------------------
        ## [6] new sigma points
        # convert mubarVec to a sigma point
        muBar = self.SigmaPoint(muBarVec[0,0], muBarVec[1,0], muBarVec[2,0], Xprev[0].mWeight, Xprev[0].cWeight)
        Xbar = self.GenerateSigmaPoints(muBar, sigmaBar)

        if self.debug: 
            print("\nXbar: \n", Xbar)
            input()

        #------------------------------------------------------------------
        ## [7] compute expected measurements (Z_t)
        Zbar = self.ComputeExpSensor(Xbar)

        if self.debug: 
            print("\nZbar: \n", Zbar)
            input()

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
        
        z_front = sensor_readings[0]
        z_right = sensor_readings[1] 
        z_left = sensor_readings[2]

        z_front_adj = (z_front/self.FAR_READING)*0.8 + 0.05
        z_right_adj = (z_right/self.FAR_READING)*0.8 + 0.05
        z_left_adj = (z_left/self.FAR_READING)*0.8 + 0.05

        self.Q_t = np.array([[z_front_adj, 0, 0],[0, z_right_adj, 0],[0, 0, z_left_adj]])
        s_t += self.Q_t # add additive sensor noise

        #------------------------------------------------------------------
        ## [10] compute cross coveriance Sigma bar_x,z
        sigmaBar_XZ = np.zeros((self.n, len(self.sensor_orientation)))
        for i in range(self.numSigPoints):
            # sigmaBar_XZ += Xprev[i].cWeight * np.matmul((Xbar[i].toVec() - muBarVec), np.transpose(Zbar[:,[i]] - zBarVec))
            # sigmaBar_XZ += Xprev[i].cWeight * np.matmul(self.StateVecDiff(Xprev[i].toVec(), muBarVec), np.transpose(Zbar[:,[i]] - zBarVec))
            sigmaBar_XZ = np.add(Xprev[i].cWeight * np.matmul(self.StateVecDiff(Xprev[i].toVec(), muBarVec), np.transpose(Zbar[:,[i]] - zBarVec)), sigmaBar_XZ, out = sigmaBar_XZ, casting = "unsafe")

        #------------------------------------------------------------------
        ## [11] compute Kalman Gain (K_t)
        # K_t = np.zeros((self.n, self.n))
        s_tInv = np.linalg.inv(s_t)
        K_t = np.matmul(sigmaBar_XZ, s_tInv)

        # print("\n\nKalman Gain:\n", K_t)

        #------------------------------------------------------------------
        ## [12] update state estimation (mu)
        # convert sensor readings to numpy array

        z = np.array([[sensor_readings[0]], [sensor_readings[1]], [sensor_readings[2]]])
        mu = self.StateVecDiff(muBarVec, -np.matmul(K_t, (z - zBarVec)))

        #------------------------------------------------------------------
        ## [13] update covariance estimation (sigma)
        sigma = sigmaBar - np.matmul(K_t, np.matmul(s_t, np.transpose(K_t)))

        #------------------------------------------------------------------

        # update state and covariance and save sigma points
        self.state.set_state(mu[0,0], mu[1,0], mu[2,0])
        self.covariance = sigma
        self.sigmaPoints = Xbar

        # print("\n Covariance:\n", self.covariance, "\n")

        return self.state

    def StateVecDiff(self, vec1, vec2):

        diff = vec1-vec2

        # wrap angle
        diff[2] = util.angle_wrap(diff[2])
        return diff

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
        def __init__(self, x, y, theta, meanWeight, covarianceWeight):
            self.x = x
            self.y = y
            self.theta = theta
            self.mWeight = meanWeight
            self.cWeight = covarianceWeight

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
            return E160_UKF.SigmaPoint(self.x, self.y, self.theta, self.mWeight, self.cWeight)
