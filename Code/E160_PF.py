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
        self.numParticles = 100
        self.numRandParticles = 0
        
        # maybe should just pass in a robot class?
        self.robotWidth = robotWidth
        self.radius = robotWidth/2
        self.wheel_radius = wheel_radius
        self.encoder_resolution = encoder_resolution
        self.FAR_READING = 3
        
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
        self.state = E160_state()
        self.state.set_state(0,0,0)

        # TODO: change this later
        self.map_maxX = 4.0
        self.map_minX = 0
        self.map_maxY = 4.0
        self.map_minY = 0
        self.InitializeParticles()
        self.last_encoder_measurements = [0,0]

    def InitializeParticles(self):
        ''' Populate self.particles with random Particle 
            Args:
                None
            Return:
                None'''
        self.particles = []
        for i in range(0, self.numParticles):
            # self.SetRandomStartPos(i)
            startPos = E160_state(0.4375,3.1,-math.pi/2)
            self.SetKnownStartPos(i, startPos)
            
    def SetRandomStartPos(self, i):
        xPos = random.uniform(self.map_minX, self.map_maxX)
        yPos = random.uniform(self.map_minY, self.map_maxY)
        theta = random.uniform(-math.pi, math.pi)
        self.particles.append(self.Particle(xPos,yPos,theta,1))

    def SetKnownStartPos(self, i, startPos):
        self.particles.append(self.Particle(startPos.x,startPos.y,startPos.theta,1))
            
    def LocalizeEstWithParticleFilterEncoder(self, encoder_measurements, sensor_readings):
        ''' Localize the robot with particle filters. Call everything
            Args: 
                econder_measurements([float, float]): encoder reading measurements
                sensor_readings([float, float, float]): sensor readings from range fingers
            Return:
                None'''
        print(sensor_readings)

        # prevent sudden simulation jumps
        if self.start:
                self.last_encoder_measurements = encoder_measurements
                self.start = False

        # encoder difference
        encoder_delta_left = encoder_measurements[0] - self.last_encoder_measurements[0]
        encoder_delta_right = encoder_measurements[1] - self.last_encoder_measurements[1]
        
        # update previous encoder values
        self.last_encoder_measurements[0] = encoder_measurements[0]
        self.last_encoder_measurements[1] = encoder_measurements[1]

        self.particle_weight_sum = 0

        for i in range(self.numParticles):

            # encoder_delta_left += np.random.normal(0, self.encoder_sigma)
            # encoder_delta_right += np.random.normal(0, self.encoder_sigma)

            # self.encoder_sigma = 0.1
            # encoder_delta_left *= np.random.normal(1, self.encoder_sigma)
            # encoder_delta_right *= np.random.normal(1, self.encoder_sigma)

            delta_s_left = float(encoder_delta_left)*(math.pi*self.wheel_radius*2)/self.encoder_resolution
            delta_s_right = float(encoder_delta_right)*(math.pi*self.wheel_radius*2)/self.encoder_resolution

            # add process noise
            delta_s_left *= np.random.normal(1, self.odom_sigma)
            delta_s_right *= np.random.normal(1, self.odom_sigma)

            delta_s = (delta_s_left + delta_s_right)/2
            delta_theta = (delta_s_left - delta_s_right)/(2*self.radius)

            self.Propagate(delta_s, delta_theta, i)
            self.particles[i].set_weight(self.CalculateWeight(sensor_readings, self.environment.walls, self.particles[i]))
            self.particle_weight_sum += self.particles[i].weight

        # normalize particle weights
        for i in range(self.numParticles):
            self.particles[i].weight *= 1.0/self.particle_weight_sum

        # check if resampling is required
        weights = [particle.weight for particle in self.particles]
        CV = np.var(weights)/(np.mean(weights)**2)        
        ESS = 1.0/(1.0 + CV)

        if (ESS < self.sampling_threshold):
            print("Resampling")
            # self.Resample()
            self.ResampleLowVar()
                
        return self.GetEstimatedPos()

    def LocalizeEstWithParticleFilter(self, state_odo, delta_s, delta_theta, sensor_readings):
        ''' Localize the robot with particle filters. Call everything
            Args: 
                delta_s (float): change in distance as calculated by odometry
                delta_heading (float): change in heading as calcualted by odometry
                sensor_readings([float, float, float]): sensor readings from range fingers
            Return:
                None'''
        print(sensor_readings)
        self.particle_weight_sum = 0

        for i in range(self.numParticles):
            # propogate and weight each particle based on sensor reading
            
            # delta_s_new = delta_s * np.random.normal(1, self.odom_xy_sigma)
            # delta_theta_new = delta_theta * np.random.normal(1, self.odom_heading_sigma)
            
            delta_s_new = delta_s + np.random.normal(0, self.odom_xy_sigma)
            delta_theta_new = delta_theta + np.random.normal(0, self.odom_heading_sigma)

            self.Propagate(delta_s_new, delta_theta_new, i)
            self.particles[i].set_weight(self.CalculateWeight(sensor_readings, self.environment.walls, self.particles[i]))
            self.particle_weight_sum += self.particles[i].weight

        # normalize particle weights
        for i in range(self.numParticles):
            self.particles[i].weight *= 1.0/self.particle_weight_sum

        # check if resampling is required
        weights = [particle.weight for particle in self.particles]
        CV = np.var(weights)/(np.mean(weights)**2)        
        ESS = 1.0/(1.0 + CV)

        if (ESS < self.sampling_threshold):
            print("Resampling")
            # self.Resample()
            self.ResampleLowVar()

        return self.GetEstimatedPos()

    def Propagate(self, delta_s, delta_theta, i):
        '''Propagate all the particles from the last state with odometry readings
            Args:
                delta_s (float): distance traveled based on odometry
                delta_theta(float): change in heading based on odometry
            return:
                nothing'''

        state = self.particles[i]

        delta_x = delta_s * math.cos(state.theta + delta_theta/2)
        delta_y = delta_s * math.sin(state.theta + delta_theta/2)

        # minus because robot is backwards
        self.particles[i].set_state(state.x - delta_x, state.y - delta_y, util.angle_wrap(state.theta + delta_theta))        
        
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
            measurementNoise = eta*norm.pdf(normalizedMeasurement)

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

    def Resample(self):
        '''Resample the particles systematically
            Args:
                None
            Return:
                None'''
        
        # self.particle_weight_sum = 0
        # for i in range(self.numParticles):
        #     self.particle_weight_sum += self.particles[i].weight
        
        newParticles = []

        for i in range(self.numParticles):
            r = random.uniform(0, self.particle_weight_sum)
            j = 0
            wsum = self.particles[j].weight
            while(wsum < r):
                j += 1
                wsum += self.particles[j].weight
            particle = self.particles[j]
            newParticles.append(self.Particle(particle.x, particle.y, particle.theta, 1.0/self.numParticles))
        
        # self.particle_weight_sum = 1.0

        # # include some purely random particles
        # for i in range(self.numRandParticles):
        #     xPos = random.uniform(self.map_minX, self.map_maxX)
        #     yPos = random.uniform(self.map_minY, self.map_maxY)
        #     theta = random.uniform(-math.pi, math.pi)
        #     newParticles.append(self.Particle(xPos,yPos,theta,1.0/self.numParticles))
            
        self.particles = newParticles

    def ResampleLowVar(self):
        '''Resample the particles systematically
            Args:
                None
            Return:
                None'''

        newParticles = []

        r = random.uniform(0, 1.0/self.numParticles)
        c = self.particles[0].weight
        i = 0

        for j in range(self.numParticles):
            U = r + (j)*(1.0/self.numParticles)
            
            while U > c:
                i = i + 1
                c += self.particles[i].weight
            
            particle = self.particles[i]
            newParticles.append(self.Particle(particle.x, particle.y, particle.theta, 1.0))

        self.particles = newParticles

    def GetEstimatedPos(self):
        ''' Calculate the mean of the particles and return it 
            Args:
                None
            Return:
                None'''
        
        xSum = 0
        ySum = 0
        sinSum = 0
        cosSum = 0

        totalWeight = 0
        for i in range(self.numParticles):
            totalWeight += self.particles[i].weight

        for i in range(self.numParticles):
            weight = self.particles[i].weight
            xSum += self.particles[i].x * (weight/totalWeight)
            ySum += self.particles[i].y * (weight/totalWeight)
            sinSum += math.sin(self.particles[i].theta) * (weight/totalWeight)
            cosSum += math.cos(self.particles[i].theta) * (weight/totalWeight)

        self.state.set_state(xSum, ySum, math.atan2(sinSum, cosSum))
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

    class Particle(E160_state):
        def __init__(self, x, y, theta, weight):
            self.x = x
            self.y = y
            self.theta = theta
            self.weight = weight

        def __str__(self):
            return str(self.x) + " " + str(self.y) + " " + str(self.theta) + " " + str(self.weight)
        
        def set_weight(self, newWeight):
            self.weight = newWeight
