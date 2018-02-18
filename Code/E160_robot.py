from E160_state import *
import math
import datetime
import logging
import csv

class E160_robot:

    def __init__(self, environment, address, robot_id):
        self.environment = environment
        
        self.state_est = E160_state()
        self.state_est.set_state(0,0,0)
        
        self.state_des = E160_state()
        self.state_des.set_state(0,0,0)

        self.state_error = E160_state()
        self.state_error.set_state(0,0,0)

        self.previous_state_error = []

        self.v = 0.05
        self.w = 0.1
        # self.radius = 0.12
        # self.wheel_radius = 0.03
        self.radius = 0.14149/2
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
        self.range_measurements = [0,0,0]
        self.last_simulated_encoder_R = 0
        self.last_simulated_encoder_L = 0

        self.front_dist = 0
        self.left_dist = 0
        self.right_dist = 0

        self.control_effort = 0
        self.error = 0
        self.move_forward_error = 0

        self.time_step = 0.1

        self.start = True

    def update(self, deltaT):

        # get sensor measurements
        self.encoder_measurements, self.range_measurements = self.update_sensor_measurements(deltaT)

        # update robot state
        [self.front_dist, self.right_dist, self.left_dist] = self.range_measurements

        # localize
        self.state_est = self.localize(self.state_est, self.encoder_measurements, self.range_measurements)

        # call motion planner
        #self.motion_planner.update_plan()

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

            # solution is of the form y = c1 + c2/(x^c3), where the coefficients were
            # calculated in MATLAB
            c = [-17.5818421751112, 7402.73728818712, 0.791041234703247]

            # convert range_measurements to cm using non linear inverse fit
            for i in range(len(range_measurements)):
                
                x = range_measurements[i]

                range_measurements[i] = c[0] + c[1]/(x**c[2])

            # keep track of measurement
            if len(self.last_measurements) < self.MAX_PAST_MEASUREMENTS:
            	self.last_measurements.append(range_measurements)

            else: 
            	self.last_measurements.pop(0)
            	self.last_measurements.append(range_measurements)

            # update current measurement to robot state
            self.currentRangeMeasurement = range_measurements

        # obtain sensor measurements !!!!!! Chris
        elif self.environment.robot_mode == "SIMULATION MODE":
            encoder_measurements = self.simulate_encoders(self.R, self.L, deltaT)
            range_measurements = [0,0,0]

        return encoder_measurements, range_measurements


    def localize(self, state_est, encoder_measurements, range_measurements):
        
        delta_s, delta_theta = self.update_odometry(encoder_measurements)
        
        state_est = self.update_state(self.state_est, delta_s, delta_theta)

        return state_est


    def update_control(self, range_measurements):

        if self.environment.control_mode == "MANUAL CONTROL MODE":
            R = self.manual_control_right_motor
            L = self.manual_control_left_motor

        elif self.environment.control_mode == "AUTONOMOUS CONTROL MODE":

            KPGain = 2
            KIGain = .1
            desiredDistance = 30

            integralError = [x[0] - desiredDistance for x in self.last_measurements]
            
            self.error = range_measurements[0] - desiredDistance
            self.control_effort = KPGain * self.error + KIGain*sum(integralError)

            R = self.control_effort
            L = self.control_effort

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
        gain = 10
        right_encoder_measurement = -int(R*gain*deltaT) + self.last_simulated_encoder_R
        left_encoder_measurement = -int(L*gain*deltaT) + self.last_simulated_encoder_L
        self.last_simulated_encoder_R = right_encoder_measurement
        self.last_simulated_encoder_L = left_encoder_measurement
        
        print("simulate_encoders", R, L, right_encoder_measurement, left_encoder_measurement)
        return [left_encoder_measurement, right_encoder_measurement]


    def make_headers(self):

        with open(self.file_name, 'a+') as log_file:
            csv_log = csv.writer(log_file)
            # with open(self.file_name, 'rb') as logfile:
            header = ['FDist', 'LDist', 'RDist', \
                     'XEst', 'YEst', 'ThetaEst', \
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
            self.manual_control_right_motor = float(R)/500
            self.manual_control_left_motor = float(L)/500


    def update_odometry(self, encoder_measurements):

        delta_s = 0
        delta_theta = 0

        encoder_delta_left = encoder_measurements[0] - self.last_encoder_measurements[0]
        encoder_delta_right = encoder_measurements[1] - self.last_encoder_measurements[1]

        # check for sudden encoder jumps (should only occur during start up)
        if encoder_delta_left > 1000 or encoder_delta_left > 1000:
            delta_s = 0
            delta_theta = 0
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
        state.set_state(state.x - delta_x, state.y - delta_y, state.theta + delta_theta)

        self.state_error = self.state_des - self.state_est

        # keep track of previous error
        if len(self.previous_state_error) < self.MAX_PAST_MEASUREMENTS:
                self.previous_state_error.append(self.state_error)
        else: 
            self.previous_state_error.pop(0)
            self.previous_state_error.append(self.state_error)

        return state