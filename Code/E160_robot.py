from E160_state import *
from E160_PF import *
from E160_UKF import *

import util
import math
import datetime
import logging
import csv

class E160_robot:

    def __init__(self, environment, address, robot_id):
        self.environment = environment
        
        # estimated state
        self.state_est = E160_state()
        self.state_est.set_state(0,0,0)

        self.state_est_prev = E160_state()
        self.state_est_prev.set_state(0,0,0)
        
        # desired state
        self.state_des = E160_state()
        self.state_des.set_state(0,0,0)

        # state error (des-est)
        self.state_error = E160_state()
        self.state_error.set_state(0,0,0)

        self.state_draw = E160_state()
        self.state_draw.set_state(0,0,0)

        self.state_odo = E160_state()
        self.state_odo.set_state(0.875/2,3.1,-math.pi/2) # real position for simulation
        # self.state_odo.set_state(0,0,0) # real position for simulation

        # dimension of robot state space
        self.DIM = 3

        # delete?
        self.previous_state_error = []

        self.R = 0
        self.L = 0

        self.v = 0.05
        self.w = 0.1
        # self.radius = 0.12
        # self.wheel_radius = 0.03
        self.radius = 0.14149/2
        self.width = 2*self.radius
        self.wheel_radius = 0.06955/2

        self.sl = 0
        self.sr = 0

        self.address = address
        self.ID = self.address.encode().__str__()[-1]

        self.MAX_PAST_MEASUREMENTS = 10

        self.last_measurements = []
        self.last_move_forward_measurements = []

        self.robot_id = robot_id

        self.manual_control_left_motor = 0
        self.manual_control_right_motor = 0

        date = datetime.datetime.now().replace(microsecond=0).__str__()
        date = date.replace(" ", "_").replace("-", "_").replace(":", "_")
        self.file_name = 'Log/Bot' + str(self.robot_id) + '_' + date
        self.file_name = self.file_name.replace(" ", "")
        self.make_headers()

        self.encoder_resolution = 1440
        self.last_encoder_measurements = [0,0]
        self.encoder_measurements = [0,0]
        self.range_measurements = [float('inf'),float('inf'),float('inf')]
        self.last_simulated_encoder_R = 0
        self.last_simulated_encoder_L = 0

        # distance sensor offsets from center of robot [front, right, left]
        self.sensor_offset = [0.076, 0.076, 0.076]

        self.front_dist = 0
        self.left_dist = 0
        self.right_dist = 0

        self.control_effort = 0
        self.error = 0
        self.move_forward_error = 0

        self.time_step = 0.1
        self.start = True

        self.last_simulated_encoder_R = 0
        self.last_simulated_encoder_L = 0
            
        self.min_ptrack_dist_error = 0.03   # meters
        self.min_ptrack_ang_error = 0.05    # radians

        if self.environment.robot_mode == "SIMULATION MODE":
            #simulation gains
            self.Kpho = 1.4#1.0
            self.Kalpha = 1.5#2.5
            self.Kbeta = -1.2#-0.5
            self.max_velocity = 0.05
            self.max_ang_velocity = 0.5
        else:
            # hardware gains
            self.Kpho = 1.0#1.0
            self.Kalpha = 2.8#2.0
            self.Kbeta = -2.3#-2.5
            self.max_velocity = 0.1
            self.max_ang_velocity = 0.8
        
        
        self.point_tracked = True
        self.encoder_per_sec_to_rad_per_sec = 10

        self.path_counter = 0

        self.path = [E160_state(0.4375, 0.355, -math.pi/2), E160_state(0.4375, 0.355, 0),
                     E160_state(3, 0.355, 0), E160_state(3, 0.355, math.pi/2), E160_state(3, 1.065, math.pi/2), E160_state(3, 1.065, math.pi),
                     E160_state(1.3125, 1.065, math.pi), E160_state(1.3125, 1.065, math.pi/2), E160_state(1.3125, 3, math.pi/2), E160_state(1.3125, 3, math.pi),
                     E160_state(0.4375, 3, math.pi), E160_state(0.4375, 3, -math.pi/2)]

        # self.path = [E160_state(0.875, 0.71, -math.pi/2), E160_state(0.875, 0.71, 0),
        #              E160_state(3, 0.71, 0), E160_state(3, 0.71, -math.pi),
        #              E160_state(0.875, 0.71, -math.pi), E160_state(0.875, 0.71, math.pi/2),
        #              E160_state(1.75/2,3, math.pi/2), E160_state(0.875, 0.71, -math.pi)]
                     
        # self.path = [E160_state(0.25, -0.25, 0), E160_state(0.25, -0.25, math.pi/2),
        #             E160_state(0.25, 0.25, math.pi/2), E160_state(0.25, 0.25, math.pi),
        #             E160_state(-0.75, 0.25, math.pi), E160_state(-0.75, 0.25, 0),
        #             E160_state(0.25, 0.25, 0), E160_state(0.25, 0.25, math.pi/2), 
        #             E160_state(0.25, -0.25, math.pi/2), E160_state(0.25, -0.25, 0),
        #             E160_state(-0.75, -0.25, 0)]
        # self.path = [E160_state(0.5, 0, 1.57), E160_state(0.5, 0.5, 3.14),
        #              E160_state(0, 0.5, 3.14+1.57), E160_state(0, 0, 0),

        #              E160_state(0.25, 0.25, 1.57), E160_state(0, 0, 0),
        #              E160_state(0.25, 0, 1.57), E160_state(0, 0, 0), 
        #              E160_state(0, 0.25, 0), E160_state(0, 0, 0), 
        #              E160_state(0, 0.25, -1.57), E160_state(0, 0, 0),
        #              E160_state(-0.25, 0, 3.14), E160_state(0, 0, 0),
        #              E160_state(0.25, 0, 3.14), E160_state(0, 0, 0)]

        # self.path = [E160_state(0, 0, 1.57), E160_state(0, 0, 0),
        #              E160_state(0, 0, -2), E160_state(0, 0, 2),
        #              E160_state(0, 0, 3.14), E160_state(0, 0, 0),

        #              E160_state(0.5, 0, 1.57), E160_state(0.5, 0.5, 3.14),
        #              E160_state(0, 0.5, 3.14+1.57), E160_state(0, 0, 0),

        #              E160_state(0.25, 0.25, 1.57), E160_state(0, 0, 0),
        #              E160_state(0.25, 0, 1.57), E160_state(0, 0, 0), 
        #              E160_state(0, 0.25, 0), E160_state(0, 0, 0), 
        #              E160_state(0, 0.25, -1.57), E160_state(0, 0, 0),
        #              E160_state(-0.25, 0, 3.14), E160_state(0, 0, 0),
        #              E160_state(0.25, 0, 3.14), E160_state(0, 0, 0)]

        # self.path = [E160_state(0, 0, 0.2), E160_state(0, 0, -0.2), E160_state(0, 0, 0),
        #              E160_state(0, 0, 1.57), E160_state(0, 0, -1.57), E160_state(0, 0, 0), 
        #              E160_state(0, 0, 3.14), E160_state(0, 0, 0), E160_state(0, 0, -3.14), E160_state(0, 0, 2), E160_state(0, 0, -2)]

        # create filter
        self.PF = E160_PF(environment, self.width, self.wheel_radius, self.encoder_resolution)

        self.UKF = E160_UKF(environment, self.DIM, self.width, self.wheel_radius, self.encoder_resolution, initialState = self.state_odo)

    def update(self, deltaT):

        # get sensor measurements
        self.encoder_measurements, self.range_measurements = self.update_sensor_measurements(deltaT)
        [self.front_dist, self.right_dist, self.left_dist] = self.range_measurements

       # update odometry
        delta_s, delta_theta = self.update_odometry(self.encoder_measurements)

        # update simulated real position, find ground truth for simulation
        self.state_odo = self.localize(self.state_odo, delta_s, delta_theta, self.range_measurements)

        # localize with particle filter
        # self.state_est = self.PF.LocalizeEstWithParticleFilter(self.state_odo, delta_s, delta_theta, self.range_measurements)
        
        # testing
        # self.state_est = self.PF.LocalizeEstWithParticleFilterEncoder(self.encoder_measurements, self.range_measurements)
                
        # ukf testing
        delta_s_noisy = delta_s + np.random.normal(0, 0.005)  # add noise to measurements
        delta_theta_noisy = delta_theta + np.random.normal(0, 0.1)  # add noise to measurements

        self.state_est = self.UKF.LocalizeEstWithUKF(delta_s_noisy, delta_theta_noisy, self.range_measurements)

        # print(self.state_draw) 

        # to output the true location for display purposes only. 
        self.state_draw = self.state_odo

        # determine new control signals
        self.R, self.L = self.update_control(self.range_measurements)

        # send the control measurements to the robot
        self.send_control(self.R, self.L, deltaT)


    def update_sensor_measurements(self, deltaT):

        # send to actual robot !!!!!!!!! John
        if self.environment.robot_mode == "HARDWARE MODE":
            command = '$S @'
            self.environment.xbee.tx(dest_addr = self.address, data = command)

            update = self.environment.xbee.wait_read_frame()

            data = update['rf_data'].decode().split(' ')[:-1]
            data = [int(x) for x in data]
            encoder_measurements = data[-2:]
            range_measurements = data[:-2]

            # prevent encoder jumping
            if self.start:
                self.last_encoder_measurements = encoder_measurements
                self.start = False

            # solution is of the form y = (c1 + c2/(x^c3))/100, where the coefficients were
            # calculated in MATLAB
            c = [-17.5818421751112, 7402.73728818712, 0.791041234703247]

            # convert range_measurements to m using non linear inverse fit and offset its location
            for i in range(len(range_measurements)):
                
                x = range_measurements[i]

                range_measurements[i] = (c[0] + c[1]/(x**c[2]))/100.0 + self.sensor_offset[i]
        
            # # keep track of measurement
            # if len(self.last_measurements) < self.MAX_PAST_MEASUREMENTS:
            #     self.last_measurements.append(range_measurements)

            # else: 
            #     self.last_measurements.pop(0)
            #     self.last_measurements.append(range_measurements)

            # update current measurement to robot state
            self.currentRangeMeasurement = range_measurements

        # obtain sensor measurements !!!!!! Chris
        elif self.environment.robot_mode == "SIMULATION MODE":
            encoder_measurements = self.simulate_encoders(self.R, self.L, deltaT)
            sensor1 = self.simulate_range_finder(self.state_odo, self.PF.sensor_orientation[0])
            sensor2 = self.simulate_range_finder(self.state_odo, self.PF.sensor_orientation[1])
            sensor3 = self.simulate_range_finder(self.state_odo, self.PF.sensor_orientation[2])
            range_measurements = [sensor1, sensor2, sensor3]

        return encoder_measurements, range_measurements


    def localize(self, state_est, delta_s, delta_theta, range_measurements):
        state_est = self.update_state(state_est, delta_s, delta_theta)
        return state_est

    def update_control(self, range_measurements):

        if self.environment.control_mode == "MANUAL CONTROL MODE":
            R = self.manual_control_right_motor
            L = self.manual_control_left_motor

        elif self.environment.control_mode == "AUTONOMOUS CONTROL MODE":

            R, L = self.point_tracker_control()
            # KPGain = 2
            # KIGain = .1
            # desiredDistance = 30

            # integralError = [x[0] - desiredDistance for x in self.last_measurements]
            
            # self.error = range_measurements[0] - desiredDistance
            # self.control_effort = KPGain * self.error + KIGain*sum(integralError)

            # R = self.control_effort
            # L = self.control_effort

        elif self.environment.control_mode == "MOVE FORWARD MODE":
            
            KPGain = 100
            KIGain = 10
            self.move_forward_error = self.state_des.x - self.state_est.x

            # keep track of measurement
            if len(self.last_move_forward_measurements) < self.MAX_PAST_MEASUREMENTS:
                self.last_move_forward_measurements.append(self.move_forward_error)

            else: 
                self.last_move_forward_measurements.pop(0)
                self.last_move_forward_measurements.append(self.move_forward_error)

            # integralError = [x[0] - desiredDistance for x in self.last_measurements]
            
            self.control_effort = KPGain *self.move_forward_error + KIGain*sum(self.last_move_forward_measurements)
            self.control_effort = min(max(self.control_effort, -40), 40)
            # angle correction
            KpGain = 8.0
            KiGain = 3.0
            KdGain = 20

            adjustment = KpGain*self.state_error.theta + KiGain*sum([state.theta for state in self.previous_state_error]) - KdGain*(self.state_error.theta - self.previous_state_error[-1].theta)/self.time_step
            print('Adjustment: ', adjustment)
            adjustment = min(max(adjustment, -5), 5)
            print('Adjustment: ', adjustment)

            R = self.control_effort + adjustment
            L = self.control_effort - adjustment

        return R, L

    def point_tracker_control(self):

        #### Delete after implementing PF ####
        state_est = self.state_odo
        #### Delete after implementing PF ####

        # state_est = self.state_est

        # calculate state error 
        self.state_error = self.state_des-state_est
        error = self.state_error

        # stop point tracking if close enough
        if (state_est.xydist(self.state_des) < self.min_ptrack_dist_error and abs(error.theta) < self.min_ptrack_ang_error): 
            self.point_tracked = True
        else: 
            self.point_tracked = False

        if not self.point_tracked:
            alpha = util.ang_diff(math.atan2(error.y, error.x), state_est.theta)
            
            # forwards
            if alpha <= math.pi/2 and alpha >= -math.pi/2:
                if (math.sqrt(error.x**2 + error.y**2) < self.min_ptrack_dist_error):
                    rho = 0
                    alpha = 0                    
                    beta = -error.theta
                else:
                    rho = math.sqrt(error.x**2 + error.y**2)
                    alpha = util.ang_diff(math.atan2(error.y, error.x), state_est.theta)
                    beta = util.ang_diff(util.ang_diff(-state_est.theta, alpha), -self.state_des.theta)
                v = self.Kpho*rho

            # backwards
            else:
                if (math.sqrt(error.x**2 + error.y**2) < self.min_ptrack_dist_error):
                    rho = 0
                    alpha = 0
                    beta = -error.theta
                else:
                    rho = math.sqrt(error.x**2 + error.y**2)
                    alpha = util.ang_diff(math.atan2(-error.y, -error.x), state_est.theta)
                    beta = util.ang_diff(util.ang_diff(-state_est.theta, alpha), -self.state_des.theta)
                v = -self.Kpho*rho


            w = self.Kalpha*alpha + self.Kbeta*beta
            
            # limit maximum linear and angular velocity
            w = max(min(w, self.max_ang_velocity), -self.max_ang_velocity)
            v = max(min(v, self.max_velocity), -self.max_velocity)
            
            # compute wheel angular velocities
            w2 = ((v/self.radius)-w)/-2
            w1 = w - w2

            desiredWheelSpeedR = 2*self.radius*w1/self.wheel_radius * self.encoder_per_sec_to_rad_per_sec
            desiredWheelSpeedL = -2*self.radius*w2/self.wheel_radius * self.encoder_per_sec_to_rad_per_sec

        else:            
            self.point_tracked = False
            self.state_des = self.path[self.path_counter]
            self.path_counter = (self.path_counter+1)%len(self.path)
            desiredWheelSpeedR = 0
            desiredWheelSpeedL = 0

        # make the simulation faster
        if self.environment.robot_mode == "SIMULATION MODE":
            simGain = 10
            desiredWheelSpeedR *= simGain
            desiredWheelSpeedL *= simGain

        return desiredWheelSpeedR,desiredWheelSpeedL

    def send_control(self, R, L, deltaT):

        # send to actual robot !!!!!!!!
        if self.environment.robot_mode == "HARDWARE MODE":
            if (L < 0):
                LDIR = 0
            else:
                LDIR = 1

            if (R < 0):
                RDIR = 0
            else:
                RDIR = 1
            RPWM = int(abs(R))
            LPWM = int(abs(L))

            command = '$M ' + str(LDIR) + ' ' + str(LPWM) + ' ' + str(RDIR) + ' ' + str(RPWM) + '@'
            self.environment.xbee.tx(dest_addr = self.address, data = command)


        # # simulate kinematics
        # elif self.environment.robot_mode == "SIMULATION MODE":
        #     self.state = self.update_state(deltaT, self.state, R, L)

    def simulate_encoders(self, R, L, deltaT):
        right_encoder_measurement = -int(R*self.encoder_per_sec_to_rad_per_sec*deltaT) + self.last_simulated_encoder_R
        left_encoder_measurement = -int(L*self.encoder_per_sec_to_rad_per_sec*deltaT) + self.last_simulated_encoder_L
        self.last_simulated_encoder_R = right_encoder_measurement
        self.last_simulated_encoder_L = left_encoder_measurement
        
        #print "simulate_encoders", R, L, right_encoder_measurement, left_encoder_measurement
        return [left_encoder_measurement, right_encoder_measurement]

    def simulate_range_finder(self, state, sensorT):
        '''Simulate range readings, given a simulated ground truth state'''
        p = self.PF.Particle(state.x, state.y, state.theta, 0)

        return self.PF.FindMinWallDistance(p, self.environment.walls, sensorT)

    def make_headers(self):

        with open(self.file_name, 'a+') as log_file:
            csv_log = csv.writer(log_file)
            # with open(self.file_name, 'rb') as logfile:
            header = ['FDist', 'LDist', 'RDist', \
                     'XEst', 'YEst', 'ThetaEst', \
                     'XOdo', 'YOdo', 'ThetaOdo', \
                     'XDes', 'YDes', 'ThetaDes', \
                     'XError', 'YError', 'ThetaError', \
                     'SL', 'SR', 'RW', 'LW']

            csv_log.writerow(header)

    def log_data(self):

        with open(self.file_name, 'a+') as log_file:
            csv_log = csv.writer(log_file)
            # with open(self.file_name, 'rb') as logfile:
            data = [self.front_dist, self.left_dist, self.right_dist, \
                    self.state_est.x, self.state_est.y, self.state_est.theta, \
                    self.state_odo.x, self.state_odo.y, self.state_odo.theta, \
                    self.state_des.x, self.state_des.y, self.state_des.theta, \
                    self.state_error.x, self.state_error.y, self.state_error.theta, \
                    self.sl, self.sr, \
                    self.manual_control_right_motor, self.manual_control_left_motor]
            csv_log.writerow(data)

    def set_manual_control_motors(self, R, L):

        if self.environment.robot_mode == "HARDWARE MODE":
            self.manual_control_right_motor = int(R*256/100)
            self.manual_control_left_motor = int(L*256/100)
        else:
            self.manual_control_right_motor = float(R)
            self.manual_control_left_motor = float(L)

    def update_odometry(self, encoder_measurements):

        delta_s = 0
        delta_theta = 0

        encoder_delta_left = encoder_measurements[0] - self.last_encoder_measurements[0]
        encoder_delta_right = encoder_measurements[1] - self.last_encoder_measurements[1]

        # check for sudden encoder jumps (should only occur during start up)
        if encoder_delta_left > 1000 or encoder_delta_left > 1000:
            delta_s = 0
            delta_theta = 0

            delta_s_left = 0
            delta_s_right = 0
        else:
            delta_s_left = float(encoder_delta_left)*(math.pi*self.wheel_radius*2)/self.encoder_resolution
            delta_s_right = float(encoder_delta_right)*(math.pi*self.wheel_radius*2)/self.encoder_resolution

            delta_s = (delta_s_left + delta_s_right)/2
            delta_theta = (delta_s_left - delta_s_right)/(2*self.radius)


        # update last encoder measurement
        self.last_encoder_measurements = encoder_measurements

        self.sl += delta_s_left
        self.sr += delta_s_right

        return delta_s, delta_theta 
     
    def update_state(self, state, delta_s, delta_theta):
        
        delta_x = delta_s * math.cos(state.theta + delta_theta/2)
        delta_y = delta_s * math.sin(state.theta + delta_theta/2)

        # minus because robot is backwards
        state.set_state(state.x - delta_x, state.y - delta_y, util.angle_wrap(state.theta + delta_theta))

        self.state_error = self.state_des - self.state_est

        # keep track of previous error
        if len(self.previous_state_error) < self.MAX_PAST_MEASUREMENTS:
                self.previous_state_error.append(self.state_error)
        else: 
            self.previous_state_error.pop(0)
            self.previous_state_error.append(self.state_error)

        return state

