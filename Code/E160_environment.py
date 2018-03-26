from E160_robot import *
from E160_state import *
from E160_wall import *
import serial
import time
from xbee import XBee


class E160_environment:


    def __init__(self):
        self.width = 4.5
        self.height = 2.5

        # set up walls, putting top left point first
        self.walls = []

        self.walls.append(E160_wall([-1, 0, 0, 0],"horizontal"))
        self.walls.append(E160_wall([-1, 0.5, 0, 0.5],"horizontal"))
        # self.walls.append(E160_wall([-1, -0.5, 0.75, -0.5],"horizontal"))
        self.walls.append(E160_wall([-1, -0.5, 0, -0.5],"horizontal"))
        self.walls.append(E160_wall([1, -1, 1, 1],"vertical"))

        # outer walls
        self.walls.append(E160_wall([-1, -1.25, -1, 1.25],"vertical"))
        self.walls.append(E160_wall([-1, -1.25, 1, -1],"horizontal"))
        self.walls.append(E160_wall([-1, 1.25, 1, 1],"horizontal"))
        # self.walls.append(E160_wall([-1.0, -1, -1.0, 1],"vertical"))
        # self.walls.append(E160_wall([-1.0, -1, -1.0, 1],"vertical"))

        # self.walls.append(E160_wall([-0.5, -0.5, -0.5, 0.5],"vertical"))
        # self.walls.append(E160_wall([0.5, -0.5, 0.5, 0.5],"vertical"))
        # self.walls.append(E160_wall([-0.5, 0.5, 0.5, 0.5],"horizontal"))

        # create vars for hardware vs simulation
        self.robot_mode = "SIMULATION MODE"#"SIMULATION MODE" or "HARDWARE MODE"
        self.control_mode = "MANUAL CONTROL MODE"

        # setup xbee communication
        if (self.robot_mode == "HARDWARE MODE"):

            try:
                self.serial_port = serial.Serial('/dev/tty.usbserial-DN02Z6QQ', 9600)
                self.xbee = XBee(self.serial_port)
                print("Setting up serial port")
            except:
                print("Serial port/XBee not found")


        # Setup the robots
        self.num_robots = 1
        self.robots = []
        for i in range (0,self.num_robots):

            # TODO: assign different address to each bot
            r = E160_robot(self, '\x00\x0C', i)
            self.robots.append(r)

    def update_robots(self, deltaT):
        # loop over all robots and update their state
        for r in self.robots:

            # set the control actuation
            r.update(deltaT)


    def log_data(self):
        # loop over all robots and update their state
        for r in self.robots:
            r.log_data()

    def quit(self):
        self.xbee.halt()
        self.serial.close()
