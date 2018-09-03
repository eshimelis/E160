from E160_robot import *
from E160_state import *
from E160_wall import *
import serial
import time
from xbee import XBee


class E160_environment:


    def __init__(self):
        # self.width = 4.5
        # self.height = 2.5

        self.width = 9
        self.height = 5

        self.INtoCM = 0.0254

        # set up walls, putting top left point first
        self.walls = []

        # self.walls.append(E160_wall([-1, 0, 0, 0]))
        # self.walls.append(E160_wall([-1, 0.5, 0, 0.5]))
        # self.walls.append(E160_wall([-1, -0.5, 0, -0.5]))
        # self.walls.append(E160_wall([1, -1, 1, 1]))

        # outer walls
        # self.walls.append(E160_wall([-1, -1.25, -1, 1.25]))
        # self.walls.append(E160_wall([-1, -1.25, 1, -1]))
        # self.walls.append(E160_wall([-1, 1.25, 1, 1]))
        
        ### self.walls.append(E160_wall([-0.5, -0.5, -0.5, 0.5],"vertical"))
        ### self.walls.append(E160_wall([0.5, -0.5, 0.5, 0.5],"vertical"))
        ### self.walls.append(E160_wall([-0.5, 0.5, 0.5, 0.5],"horizontal"))

        ## South Maze

        # outer walls
        # self.walls.append(E160_wall([-2, -2, -2, 2]))
        # self.walls.append(E160_wall([2, -2, 2, 2]))
        # self.walls.append(E160_wall([-2, -2, 2, -2]))
        # self.walls.append(E160_wall([-2, 2, 2, 2]))

        # # 
        # self.walls.append(E160_wall([-2, 2 - 44*self.INtoCM, -2 + 14.5*self.INtoCM, 2 - 44*self.INtoCM]))
        # self.walls.append(E160_wall([-2 + 22*self.INtoCM, 2, -2 + 22*self.INtoCM, 2 - 28*self.INtoCM]))
        # self.walls.append(E160_wall([-2 + (6.5+22)*self.INtoCM, 2, -2 + (6.5+22)*self.INtoCM, 2 - 17*self.INtoCM]))
        # self.walls.append(E160_wall([-2 + (3.5+22)*self.INtoCM, 2-17*self.INtoCM, -2 + (3.5+22)*self.INtoCM, 2 - 28*self.INtoCM]))
        ## self.walls.append(E160_wall([-2 + (3.5+22)*self.INtoCM, 2-17*self.INtoCM, -2 + (3.5+22)*self.INtoCM, 2 - 28*self.INtoCM]))

        # self.walls.append(E160_wall([-2, 2 - 44*self.INtoCM - 28*self.INtoCM, 2, 2 - 44*self.INtoCM - 28*self.INtoCM]))
        
        ## corner test
        # self.walls.append(E160_wall([1, -1, 1, 1]))
        # self.walls.append(E160_wall([-1, -1, 1, -1]))
        # self.walls.append(E160_wall([-1, -1+48*self.INtoCM, 1, -1+48*self.INtoCM]))

        ## Parsons corner map [x1, y1, x2, y2] (Next to Lab/Room 2391)
        wall1a = [0, 0, 0, 0.94]
        wall1b = [0, 1.85, 0, 4.01]

        door1a = [-0.12, 0.94, -0.12, 1.85]
        door1b = [-0.12, 0.94, 0, 0.94]
        door1c = [-0.12, 1.85, 0, 1.85]

        wall2 = [0, 0, 4, 0]
        wall3 = [1.75, 1.8, 1.75, 4]
        wall4 = [1.75, 1.8, 1.75+0.87, 1.8]
        wall5 = [1.75+0.87, 1.8, 1.75+0.87, 1.8-0.38]
        wall6 = [1.75+0.87, 1.8-0.38, 4, 1.8-0.38]

        self.walls.append(E160_wall(wall1a))
        self.walls.append(E160_wall(wall1b))
        self.walls.append(E160_wall(wall2))
        self.walls.append(E160_wall(wall3))
        self.walls.append(E160_wall(wall4))
        self.walls.append(E160_wall(wall5))
        self.walls.append(E160_wall(wall6))

        self.walls.append(E160_wall(door1a))
        self.walls.append(E160_wall(door1b))
        self.walls.append(E160_wall(door1c))

        # added obstacles
        wall7 = [1.75, 2, 0.75, 2]
        wall8 = [3, 1.8-0.38, 3, 0.5]
        wall9 = [1.5, 0, 1.5, 1]

        self.walls.append(E160_wall(wall7))
        self.walls.append(E160_wall(wall8))
        self.walls.append(E160_wall(wall9))

        # create vars for hardware vs simulation
        self.robot_mode = "SIMULATION MODE"#"SIMULATION MODE" or "HARDWARE MODE"
        self.control_mode = "MANUAL CONTROL MODE"

        if self.robot_mode == "SIMULATION MODE":
            endcap1 = [-0.01, 4, 1.75, 4]
            endcap2 = [4, 0, 4, 1.8-0.38]
            self.walls.append(E160_wall(endcap1))
            self.walls.append(E160_wall(endcap2))

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
