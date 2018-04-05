import math
import E160_state
from tkinter import *
from E160_robot import *
from PIL import Image, ImageTk
import util

class E160_graphics:

    def __init__(self, environment):
        self.environment = environment
        self.tk = Tk()

        self.north_west_frame = Frame(self.tk)
        self.north_west_frame.pack(anchor = W)

        self.scale = 200
        self.canvas = Canvas(self.tk, width=self.environment.width*self.scale, height=self.scale*self.environment.height)
        self.tk.title("E160 - Autonomous Robot Navigation")
        self.canvas.bind("<Button-1>", self.callback)
        self.canvas.pack()
        self.gui_stopped = False
        self.last_rotate_control = 0
        self.last_forward_control = 0
        self.R = 0
        self.L = 0

        # add motor control slider
        self.forward_control = Scale(self.tk, from_=-100, to=100, length  = 400,label="Forward Control",tickinterval=50, orient=HORIZONTAL)
        self.forward_control.pack(side=LEFT)

        # add rotation control slider
        self.rotate_control = Scale(self.tk, from_=-100, to=100, length  = 400,label="Rotate Control",tickinterval=50, orient=HORIZONTAL)
        self.rotate_control.pack(side=RIGHT)

        # add track point button
        self.track_point_button = Button(self.tk, text="Track Point", anchor="s", wraplength=100, command=self.track_point).pack()

        # add track point button
        self.track_waypoint_button = Button(self.tk, text="Track Waypoints", anchor="s", wraplength=100, command=self.track_waypoint).pack()

        # add stop button
        self.stop_point_button = Button(self.tk, text="Stop", anchor="s", wraplength=100, command=self.stop).pack()

        # add quit button
        self.quit_point_button = Button(self.tk, text="Quit", anchor="s", wraplength=100, command=self.quit).pack()

        # add reset button
        self.reset_point_button = Button(self.tk, text="Reset", anchor="s", wraplength=100, command=self.reset).pack()

        # add move buttons
        # self.move_button = Button(self.tk, text="Move Forward", anchor="s", wraplength=100, command=self.move_forward).pack()
        # self.move_distance = Entry(self.tk)
        # self.move_distance.pack()
        # self.goal_distance = 0

        # arrow input
        self.tk.bind('<Up>', self.up_arrow_key_input)
        self.tk.bind('<Down>', self.down_arrow_key_input)
        self.tk.bind('<Left>', self.left_arrow_key_input)
        self.tk.bind('<Right>',  self.right_arrow_key_input)
        self.tk.bind('<KeyRelease>', self.key_released)
        
        # add range sensor measurements
        self.range_sensor_var_1 = StringVar()
        self.range_sensor_var_2 = StringVar()
        self.range_sensor_var_3 = StringVar()
        self.range_sensor_label_1 = Label(self.north_west_frame, textvariable = self.range_sensor_var_1).pack()
        self.range_sensor_label_2 = Label(self.north_west_frame, textvariable = self.range_sensor_var_2).pack()
        self.range_sensor_label_3 = Label(self.north_west_frame, textvariable = self.range_sensor_var_3).pack()

        # add encoder sensor measurements
        self.encoder_sensor_var_0 = StringVar()
        self.encoder_sensor_var_1 = StringVar()
        
        self.encoder_sensor_label_0 = Label(self.north_west_frame, textvariable = self.encoder_sensor_var_0).pack()
        self.encoder_sensor_label_1 = Label(self.north_west_frame, textvariable = self.encoder_sensor_var_1).pack()

        # add range sensor measurements
        self.x = StringVar()
        self.y = StringVar()
        self.theta = StringVar()
        self.x_label = Label(self.north_west_frame, textvariable = self.x).pack()
        self.y_label = Label(self.north_west_frame, textvariable = self.y).pack()
        self.theta_label = Label(self.north_west_frame, textvariable = self.theta).pack()
       
        # add text entry for desired X
        #self.x_des_label = Label(self.north_frame, text="X desired")
        #self.x_des_label.pack()
        self.x_des_entry = Entry(self.north_west_frame, justify = RIGHT)
        self.x_des_entry.insert(10,"0.0")
        self.x_des_entry.pack()
        
        # add text entry for desired Y
        #self.y_des_label = Label(self.north_west_frame, text="Y desired")
        #self.y_des_label.pack()
        self.y_des_entry = Entry(self.north_west_frame, justify = RIGHT)
        self.y_des_entry.insert(10,"0.0")
        self.y_des_entry.pack()
        
        # add text entry for desired Theta
        #self.theta_des_label = Label(self.north_west_frame, text="Theta desired")
        #self.theta_des_label.pack()
        self.theta_des_entry = Entry(self.north_west_frame, justify = RIGHT)
        self.theta_des_entry.insert(10,"0.0")
        self.theta_des_entry.pack()

        # desired and estimated state arrow
        self.arrow_des = self.canvas.create_line(0, 0, 0, 0, tags=("des_arrow",), arrow="last", fill="orange red", width=8)
        self.arrow_odo = self.canvas.create_line(0, 0, 0, 0, tags=("odo_arrow",), arrow="last", fill="steel blue", width=4)
        self.arrow_est = self.canvas.create_line(0, 0, 0, 0, tags=("est_arrow",), arrow="last", fill="chartreuse2", width=4)

        # for testing
        # self.leftSensor = self.canvas.create_line(0, 0, 0, 0, tags=("left_sensor",), arrow="last", fill="red", width=4)
        # self.rightSensor = self.canvas.create_line(0, 0, 0, 0, tags=("right_sensor",), arrow="last", fill="blue", width=4)

        # self.canvas.bind("<ButtonPress-1>", self.on_button_press)
        # self.canvas.bind("<B1-Motion>", self.on_move_press)
        # self.canvas.bind("<ButtonRelease-1>", self.on_button_release)

        # sensor wall intersection
        self.front_wall_sensor = self.canvas.create_oval(-3,-3,3,3, fill ='orange red', tags=("front_wall_sensor"))

        # initilize particle representation
        self.particles_dot = [self.canvas.create_oval(0,0,0,0, fill ='black') for x in range(self.environment.robots[0].PF.numParticles)]

        # initilize particle representation
        self.particles_vec = [self.canvas.create_line(0, 0, 0, 0, arrow="last", fill="medium sea green", width=2) for x in range(self.environment.robots[0].PF.numParticles)]

        # draw static environment
        for w in self.environment.walls:
            self.draw_wall(w)

        # draw first robot
        for r in self.environment.robots:
            self.initial_draw_robot(r)

    # draws walls on gui
    def draw_wall(self, wall):

        wall_points = self.scale_points(wall.wall_points, self.scale)
        wall.poly = self.canvas.create_line(wall_points, fill='black', width = 6)

    def scale_points(self, points, scale):
        scaled_points = []
        for i in range(len(points)-1):

            if i % 2 == 0:
                # for x values, just multiply times scale factor to go from meters to pixels
                scaled_points.append(self.environment.width/2*scale + (points[i]-2)*scale)

                # only flip y for x,y points, not for circle radii
                scaled_points.append(self.environment.height/2*scale - (points[i+1]-2)*scale)

        return scaled_points

    def reverse_scale_points(self, points, scale):
        reverse_scaled_points = []
        for i in range(len(points)-1):

            if i % 2 == 0:
                # for x values, just multiply times scale factor to go from meters to pixels
                reverse_scaled_points.append(-self.environment.width/2 + points[i]/scale)

                # only flip y for x,y points, not for circle radii
                reverse_scaled_points.append(self.environment.height/2 - points[i+1]/scale)

        return reverse_scaled_points

    def initial_draw_robot(self, robot):
        # open image
        robot.robot_gif = Image.open("E160_robot_image.gif").convert('RGBA')

    def draw_robot(self, robot):
        
        # gif update
        robot.tkimage = ImageTk.PhotoImage(robot.robot_gif.rotate(180/3.14*robot.state_draw.theta))
        # robot.image = self.canvas.create_image(robot.state_draw.x, robot.state_draw.y, image=robot.tkimage)
        robot_points = self.scale_points([robot.state_draw.x, robot.state_draw.y], self.scale)
        # self.canvas.coords(robot.image, *robot_points)

        # leftSensorHeadingDisp = math.pi/2
        # rightSensorHeadingDisp  = -math.pi/2

        # leftSensor = E160_state(robot.state_draw.x, robot.state_draw.y, robot.state_draw.theta+leftSensorHeadingDisp)
        # rightSensor = E160_state(robot.state_draw.x, robot.state_draw.y, robot.state_draw.theta+rightSensorHeadingDisp)

        # self.update_arrow(leftSensor, 'left_sensor', 0.25)
        # self.update_arrow(rightSensor, 'right_sensor', 0.25)

        # self.update_point(robot.WallPoint, "front_wall_sensor")
        # self.front_wall_sensor = self.canvas.create_oval(0,0,0,0, fill ='black', tags=("front_wall_sensor"))

    def draw_particles(self, robot):
        
        for i in range(robot.PF.numParticles):
            self.canvas.delete(self.particles_vec[i])
            self.particles_vec[i] = self.draw_arrow(robot.PF.particles[i], 0.1)

        for i in range(robot.PF.numParticles):
            pf_point = [robot.PF.particles[i].x, robot.PF.particles[i].y]
            point = self.scale_points(pf_point, self.scale)
            self.canvas.delete(self.particles_dot[i]) 
            self.particles_dot[i] = self.canvas.create_oval(point[0] - 4, point[1] - 4, point[0] + 4, point[1] + 4, fill =  'red')

        # add estimated and desired position arrows
        self.update_arrow(robot.state_des, "des_arrow", 0.25)
        self.update_arrow(robot.state_draw, "odo_arrow", 0.25)
        self.update_arrow(robot.state_est, "est_arrow", 0.25)
        
    def update_point(self, pos, tag):
        if pos != None:
            # print(pos.x, ',', pos.y)
            [x, y] = self.scale_points([pos.x, pos.y], self.scale)
            # print(x, ",", y)
            # self.canvas.delete(self.front_wall_sensor)
            self.front_wall_sensor = self.canvas.create_oval(x - 3, y - 3, x + 3, y + 3, fill =  'red')
            # self.canvas.move(tag, x, y)
            # self.canvas.tag_raise(tag)

    def update_arrow(self, state, tag, length):
        startx = state.x
        starty = state.y
        endx = state.x + length*math.cos(state.theta)
        endy = state.y + length*math.sin(state.theta)
        points = [startx, starty, endx, endy]
        [startx, starty, endx, endy] = self.scale_points(points, self.scale)
        self.canvas.coords(tag, startx, starty, endx, endy)
        self.canvas.tag_raise(tag)

    def draw_arrow(self, state, length):
        startx = state.x
        starty = state.y
        endx = state.x + length*math.cos(state.theta)
        endy = state.y + length*math.sin(state.theta)
        points = [startx, starty, endx, endy]
        [startx, starty, endx, endy] = self.scale_points(points, self.scale)
        return self.canvas.create_line(startx, starty, endx, endy, fill="orange red", width=2)

    def get_inputs(self):
        pass

    def track_point(self):
        self.environment.control_mode = "AUTONOMOUS CONTROL MODE"
                
        # update sliders on gui
        self.forward_control.set(0)
        self.rotate_control.set(0)
        self.last_forward_control = 0
        self.last_rotate_control = 0
        self.R = 0
        self.L = 0
        
        # draw robots
        for r in self.environment.robots:
            x_des = float(self.x_des_entry.get())
            y_des = float(self.y_des_entry.get())
            theta_des = float(self.theta_des_entry.get())
            r.state_des.set_state(x_des,y_des,theta_des)
            r.point_tracked = False

    def track_waypoint(self):
        self.environment.control_mode = "AUTONOMOUS CONTROL MODE"
                
        # update sliders on gui
        self.forward_control.set(0)
        self.rotate_control.set(0)
        self.last_forward_control = 0
        self.last_rotate_control = 0
        self.R = 0
        self.L = 0
        
        # draw robots
        for r in self.environment.robots:
            if r.path_counter <= len(r.path):
                des_state = r.path[r.path_counter]
                r.state_des.set_state(des_state.x, des_state.y, des_state.theta)
                r.path_counter += 1

    def stop(self):
        self.environment.control_mode = "MANUAL CONTROL MODE"

        # update sliders on gui
        self.forward_control.set(0)
        self.rotate_control.set(0)
        self.last_forward_control = 0
        self.last_rotate_control = 0
        self.R = 0
        self.L = 0

    def quit(self):
        self.environment.control_mode = "MANUAL CONTROL MODE"
        self.forward_control.set(0)
        self.rotate_control.set(0)
        self.gui_stopped = True

    def reset(self):
        self.environment.control_mode = "AUTONOMOUS CONTROL MODE"
        self.environment.robots[0].state_des.set_state(0, 0, 0)
        self.environment.robots[0].point_tracked = False

    def move_forward(self):
        self.environment.control_mode = "MOVE FORWARD MODE"
        self.goal_distance = self.environment.robots[0].state_est.x + float(self.move_distance.get())
        # self.forward_control.set(10)

        self.environment.robots[0].state_des.set_state(self.environment.robots[0].state_est.x + float(self.move_distance.get()), 0, 0)

    def callback(self, event):
        desired_points = self.reverse_scale_points([float(event.x), float(event.y)], self.scale)
        robot = self.environment.robots[0]
        robot.state_des.set_state(desired_points[0],desired_points[1],0)
        print("New desired robot state", robot.state_des.x, robot.state_des.y)

    def send_robot_commands(self):

        # check to see if forward slider has changed
        if abs(self.forward_control.get()-self.last_forward_control) > 0:
            self.rotate_control.set(0)
            self.last_forward_control = self.forward_control.get()
            self.last_rotate_control = 0
            self.environment.control_mode = "MANUAL CONTROL MODE"

            # extract what the R and L motor signals should be
            self.R = self.forward_control.get()
            self.L = self.forward_control.get()

        # check to see if rotate slider has changed
        elif abs(self.rotate_control.get()-self.last_rotate_control) > 0:
            self.forward_control.set(0)
            self.last_rotate_control = self.rotate_control.get()
            self.last_forward_control = 0
            self.environment.control_mode = "MANUAL CONTROL MODE"

            # extract what the R and L motor signals should be
            self.R = -self.rotate_control.get()
            self.L = self.rotate_control.get()

        # if manual mode, set motors
        if self.environment.control_mode == "MANUAL CONTROL MODE":

            # tell robot what the values should be
            robot = self.environment.robots[0]
            robot.set_manual_control_motors(self.R, self.L)

    def up_arrow_key_input(self, event):
        if self.environment.control_mode == "MANUAL CONTROL MODE":
            self.forward_control.set(20)
            self.rotate_control.set(0)
        else:
            self.forward_control.set(100)
            self.rotate_control.set(0)

    def down_arrow_key_input(self, event):
        if self.environment.control_mode == "MANUAL CONTROL MODE":
            self.forward_control.set(-20)
            self.rotate_control.set(0)
        else:
            self.forward_control.set(-100)
            self.rotate_control.set(0)

    def left_arrow_key_input(self, event):
        if self.environment.control_mode == "MANUAL CONTROL MODE":
            self.forward_control.set(0)
            self.rotate_control.set(-20)
        else:
            self.forward_control.set(0)
            self.rotate_control.set(-100)

    def right_arrow_key_input(self, event):
        if self.environment.control_mode == "MANUAL CONTROL MODE":
            self.forward_control.set(0)
            self.rotate_control.set(20)
        else:
            self.forward_control.set(0)
            self.rotate_control.set(100)
    
    def key_released(self, event):
        self.forward_control.set(0)
        self.rotate_control.set(0)

    def update_sensor_readings(self, distance):
        # update reading on gui
        outputText = "Front: " + str(distance[0]) + "\nRight: " + str(distance[1]) + "\nLeft: " + str(distance[2])
        self.distance_measurements.config(text = outputText)

    def update_labels(self):
        
        self.range_sensor_var_1.set("Front Range (m):  " + str(self.environment.robots[0].range_measurements[0]))
        self.range_sensor_var_2.set("Right Range (m):  " + str(self.environment.robots[0].range_measurements[1]))
        self.range_sensor_var_3.set("Left Range (m):  " + str(self.environment.robots[0].range_measurements[2]))
                
        self.encoder_sensor_var_0.set("Encoder 0 (m):  " + str(self.environment.robots[0].encoder_measurements[0]))
        self.encoder_sensor_var_1.set("Encoder 1 (m):  " + str(self.environment.robots[0].encoder_measurements[1]))

        self.x.set("X (m):  " + str(self.environment.robots[0].state_est.x))
        self.y.set("Y (m):  " + str(self.environment.robots[0].state_est.y))
        self.theta.set("Theta (rad):  " + str(self.environment.robots[0].state_est.theta))

    # called at every iteration of main loop
    def update(self):

        # update gui labels
        self.update_labels()

        # draw robots
        for r in self.environment.robots:
            self.draw_robot(r)
                
        # draw particles
        self.draw_particles(self.environment.robots[0])

        # draw sensors

        # update desired state arrow
        # self.update_gui_des(des_robot_state)

        # update the graphics
        self.tk.update()

        # check for gui buttons
        self.get_inputs()

        # send commands to robots
        self.send_robot_commands()

        # check for quit
        if self.gui_stopped:
            self.environment.quit()
            return False
        else:
            return True
