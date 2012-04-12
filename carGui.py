# ==============================================================================
# ANIMATED RACETRACK CONTROL
# djordje.trifunovic@gmail.com, March 2012
#
# Animation illustrating robot motion with PID control on a cyclic track.
# Based on prof. Sebastian Thrun's/Udacity team's code from March 2012 Udacity
# CS373 course (Programming a Robotic Car): Unit 5, Homework 5-4
#
# Controls:
#   ENTER: Start/stop animation.
#   MOUSE PRESS: Relocate robot
#   MOUSE DRAG: Change robot orientation
#   ESC or Window Close Button: Quit demo.
# ==============================================================================

from Tkinter import *
from math import *
import random
import numpy
from numpy.random import random_integers as rnd
from matrix import *


#----------------the maze thing-----------------------------------------------------------------

def maze(width, height, recurse=False):
    """Return a generator where each iteration is a full numpy
    array representing the next stage of maze layout.
    """
    # Only odd shapes
    height, width = (height // 2) * 2 + 1, (width // 2) * 2 + 1
    f = maze_non_recurse if not recurse else maze_recurse
    return f(width, height)

def maze_recurse(width, height):
    # Init maze as numpy array (all walls)
    Z = numpy.ones((height, width))

    def carve(y, x):
        Z[y, x] = 0
        yield Z
        # get randomized list of neighbours
        neighbours = [(x + 2, y), (x - 2, y), (x, y + 2), (x, y - 2)]
        random.shuffle(neighbours)
        for nx,ny in neighbours:
            if nx < 0 or ny < 0 or nx >= width or ny >= height:
                continue
            if Z[ny, nx] == 1:
                Z[(y + ny) / 2,(x + nx) / 2] = 0
                for m in carve(ny, nx):
                    yield m

    # choose random internal starting point
    x, y = rnd(0, width // 2 - 1) * 2 + 1, rnd(0, height // 2 - 1) * 2 + 1
    for m in carve(y, x):
        yield m

    # carve exits
    Z[1, 0] = Z[-2, -1] = 0
    yield Z

def maze_non_recurse(width, height):
    # Init maze as numpy array (all walls)
    Z = numpy.ones((height, width))
    stack = []
    # choose random internal starting point
    x, y = rnd(0, width // 2 - 1) * 2 + 1, rnd(0, height // 2 - 1) * 2 + 1
    # get randomized list of neighbours
    n = neighbours(x, y, width, height)
    while True:
        Z[y, x] = 0
        yield Z

        for nx, ny in n:
            if Z[ny, nx] == 1:
                Z[(y + ny) / 2, (x + nx) / 2] = 0
                stack.append((n, (x, y)))
                y, x = ny, nx
                n = neighbours(x, y, width, height)
                break
        else:
            try:
                n, (x, y) = stack.pop()
            except IndexError:
                break

    # carve exits
    Z[1, 0] = Z[-2, -1] = 0
    yield Z

def maze_coords(width, height):
    """Return a generator where each iteration is the x, y coord of
    the next wall (brick?) location.
    """
    # Init maze as numpy array (all walls)
    Z = numpy.ones((height, width))
    stack = []
    # choose random internal starting point
    x, y = rnd(0, width // 2 - 1) * 2 + 1, rnd(0, height // 2 - 1) * 2 + 1
    # get randomized list of neighbours
    n = neighbours(x, y, width, height)
    yield -1, -1
    while True:
        Z[y, x] = 0
        yield (x, y)

        for nx, ny in n:
            if Z[ny, nx] == 1:
                Z[(y + ny) / 2, (x + nx) / 2] = 0
                yield ((x + nx) / 2, (y + ny) / 2)

                stack.append((n, (x, y)))
                y, x = ny, nx
                n = neighbours(x, y, width, height)
                break
        else:
            try:
                n, (x, y) = stack.pop()
                yield (-1, -1)
            except IndexError:
                break

    # carve exits
    Z[1, 0] = Z[-2, -1] = 0

def neighbours(x, y, width, height):
    n = [(x + 2, y), (x - 2, y), (x, y + 2), (x, y - 2)]
    n = filter(lambda v: 0 <= v[0] < height and 0 <= v[1] < height, n)
    random.shuffle(n)
    return iter(n)


#----------------the maze thing-----------------------------------------------------------------



# ------------------------------------------------
#
# Plan class:
#

class plan:

    # --------
    # init: 
    #    creates an empty plan
    #

    def __init__(self, grid, init, goal, cost = 1):
        self.cost = cost
        self.grid = grid
        self.init = init
        self.goal = goal
        #print(goal, self.cost)
        self.make_heuristic(grid, goal, self.cost)
        self.path = []
        self.spath = []

    # --------
    #
    # make heuristic function for a grid
        
    def make_heuristic(self, grid, goal, cost):
        self.heuristic = [[0 for row in range(len(grid[0]))] 
                          for col in range(len(grid))]
        for i in range(len(self.grid)):    
            for j in range(len(self.grid[0])):
                self.heuristic[i][j] = abs(i - self.goal[0]) + \
                    abs(j - self.goal[1])



    # ------------------------------------------------
    # 
    # A* for searching a path to the goal
    #
    #

    def astar(self):


        if self.heuristic == []:
            raise ValueError, "Heuristic must be defined to run A*"
        # print self.heuristic
        # internal motion parameters
        delta = [[-1,  0], # go up
                 [ 0,  -1], # go left
                 [ 1,  0], # go down
                 [ 0,  1]] # do right


        # open list elements are of the type: [f, g, h, x, y]

        closed = [[0 for row in range(len(self.grid[0]))] 
                  for col in range(len(self.grid))]
        action = [[0 for row in range(len(self.grid[0]))] 
                  for col in range(len(self.grid))]

        closed[self.init[0]][self.init[1]] = 1


        x = self.init[0]
        y = self.init[1]
        h = self.heuristic[x][y]
        g = 0
        f = g + h

        open = [[f, g, h, x, y]]

        found  = False # flag that is set when search complete
        resign = False # flag set if we can't find expand
        count  = 0


        while not found and not resign:

            # check if we still have elements on the open list
            if len(open) == 0:
                resign = True
                print '###### Search terminated without success'
                
            else:
                # remove node from list
                open.sort()
                open.reverse()
                next = open.pop()
                x = next[3]
                y = next[4]
                g = next[1]

            # check if we are done

            if x == self.goal[0] and y == self.goal[1]:
                found = True
                # print '###### A* search successful'

            else:
                # expand winning element and add to new open list
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if x2 >= 0 and x2 < len(self.grid) and y2 >= 0 \
                            and y2 < len(self.grid[0]):
                        if closed[x2][y2] == 0 and self.grid[x2][y2] == 0:
                            g2 = g + self.cost
                            h2 = self.heuristic[x2][y2]
                            f2 = g2 + h2
                            open.append([f2, g2, h2, x2, y2])
                            closed[x2][y2] = 1
                            action[x2][y2] = i

            count += 1

        # extract the path



        invpath = []
        x = self.goal[0]
        y = self.goal[1]
        invpath.append([x, y])
        while x != self.init[0] or y != self.init[1]:
            x2 = x - delta[action[x][y]][0]
            y2 = y - delta[action[x][y]][1]
            x = x2
            y = y2
            invpath.append([x, y])

        self.path = []
        for i in range(len(invpath)):
            self.path.append(invpath[len(invpath) - 1 - i])




    # ------------------------------------------------
    # 
    # this is the smoothing function
    #

  


    def smooth(self, weight_data = 0.1, weight_smooth = 0.1, 
               tolerance = 0.000001):

        if self.path == []:
            raise ValueError, "Run A* first before smoothing path"

        self.spath = [[0 for row in range(len(self.path[0]))] \
                           for col in range(len(self.path))]
        for i in range(len(self.path)):
            for j in range(len(self.path[0])):
                self.spath[i][j] = self.path[i][j]

        change = tolerance
        while change >= tolerance:
            change = 0.0
            for i in range(1, len(self.path)-1):
                for j in range(len(self.path[0])):
                    aux = self.spath[i][j]
                    
                    self.spath[i][j] += weight_data * \
                        (self.path[i][j] - self.spath[i][j])
                    
                    self.spath[i][j] += weight_smooth * \
                        (self.spath[i-1][j] + self.spath[i+1][j] 
                         - (2.0 * self.spath[i][j]))
                    if i >= 2:
                        self.spath[i][j] += 0.5 * weight_smooth * \
                            (2.0 * self.spath[i-1][j] - self.spath[i-2][j] 
                             - self.spath[i][j])
                    if i <= len(self.path) - 3:
                        self.spath[i][j] += 0.5 * weight_smooth * \
                            (2.0 * self.spath[i+1][j] - self.spath[i+2][j] 
                             - self.spath[i][j])
                
                    change += abs(aux - self.spath[i][j])



# ------------------------------------------------
# 
# this is the particle filter class
#

class particles:

    # --------
    # init: 
    #	creates particle set with given initial position
    #

    def __init__(self, x, y, theta, 
                 steering_noise, distance_noise, measurement_noise, N = 100):
        self.N = N
        self.steering_noise    = steering_noise
        self.distance_noise    = distance_noise
        self.measurement_noise = measurement_noise
        
        self.data = []
        for i in range(self.N):
            r = Robot()
            r.set(x, y, theta)
            r.set_noise(steering_noise, distance_noise, measurement_noise)
            self.data.append(r)


    # --------
    #
    # extract position from a particle set
    # 
    
    def get_position(self):
        x = 0.0
        y = 0.0
        #orientation = 0.0
        ox = 0.0
        oy = 0.0

        for i in range(self.N):
            x += self.data[i].x
            y += self.data[i].y
            # orientation is tricky because it is cyclic. By normalizing
            # around the first particle we are somewhat more robust to
            # the 0=2pi problem
            #orientation += (((self.data[i].orientation
            #                  - self.data[0].orientation + pi) % (2.0 * pi)) 
            #                + self.data[0].orientation - pi)

            # CW Method to calculated orrientation
            # Average vectors correctly (but at the added cost of
            # a lot of trig function overhead)

            ox += cos(self.data[i].orientation)
            oy += sin(self.data[i].orientation)

        #orientation = orientation / self.N
        orientation = atan2(oy, ox) % (2*pi)
        
        return [x / self.N, y / self.N, orientation]

    # --------
    #
    # motion of the particles
    # 
    def move(self, steer, speed):
        newdata = []

        for i in range(self.N):
            r = self.data[i].move(steer, speed)
            newdata.append(r)
        self.data = newdata
        
    def move_with_grid(self, grid, steer, speed):
        newdata = []

        for i in range(self.N):
            r = self.data[i].move(grid, steer, speed)
            newdata.append(r)
        self.data = newdata

    # --------
    #
    # sensing and resampling
    # 

    def sense(self, Z):
        w = []
        for i in range(self.N):
            w.append(self.data[i].measurement_prob(Z))

        # resampling (careful, this is using shallow copy)
        p3 = []
        index = int(random.random() * self.N)
        beta = 0.0
        mw = max(w)

        for i in range(self.N):
            beta += random.random() * 2.0 * mw
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % self.N
            p3.append(self.data[index])
        self.data = p3


# ------------------------------------------------
#
# Robot class:
#
class Robot:
    # --------
    # init:
    #   creates robot and initializes location/orientation to 0, 0, 0
    
    def __init__(self, length = 30.0, width = None,
                 wheel_length = None, wheel_width = None,
                 length_margin = None, width_margin = None,
                 max_steering_angle = pi / 4.0, 
                 color="red", wheel_color="black"):
        # default argument values
        if width == None:
            width = 0.4*length
        if wheel_length == None:
            wheel_length = 0.2*length
        if wheel_width == None:
            wheel_width = 0.2*wheel_length
        if length_margin == None:
           length_margin = wheel_length
        if width_margin == None:
            width_margin = wheel_length/2.0
        
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0 # heading east
        self.max_steering_angle = max_steering_angle
        self.steering = 0.0 # without drift
        self.steering_drift = 0.0
        self.steering_noise = 0.0
        self.measurement_noise = 0.0
        self.distance_noise = 0.0
        self.length = length # distance between axles
        self.width = width # distance between wheel centers (length of an axle)
        self.wheel_length = wheel_length
        self.wheel_width = wheel_width
        self.length_margin = length_margin # distance from wheel center to the bumper
        self.width_margin = width_margin # distance from wheel center to the side
        self.color = color
        self.wheel_color = wheel_color
        self.measurement_range = 200.0
    
    # --------
    # copy:
    #   deep copy
    
    def copy(self):
        result = Robot()
        result.x = self.x
        result.y = self.y
        result.orientation = self.orientation
        result.max_steering_angle = self.max_steering_angle
        result.steering = self.steering
        result.steering_drift = self.steering_drift
        result.steering_noise = self.steering_noise
        result.distance_noise = self.distance_noise
        result.measurement_noise = self.measurement_noise
        result.length = self.length
        result.width = self.width
        result.wheel_length = self.wheel_length
        result.wheel_width = self.wheel_width
        result.length_margin = self.length_margin
        result.width_margin = self.width_margin
        result.color = self.color
        result.wheel_color = self.wheel_color
        return result
    
    # --------
    # set:
    #   sets a robot position and orientation
    #   (robot position is determined by the center of rear axle)

    def set(self, new_x, new_y, new_orientation):
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation) % (2.0 * pi)
        

    # --------
    # set_noise:
    #   sets steering and distance noise

    def set_noise(self, new_s_noise, new_d_noise, new_m_noise):
        # this is often useful in particle filters
        self.steering_noise = float(new_s_noise)
        self.distance_noise = float(new_d_noise)
        self.measurement_noise = float(new_m_noise)

    # --------
    # set_steering_drift:
    #   sets the systematical steering drift parameter

    def set_steering_drift(self, drift):
        self.steering_drift = drift

    # --------
    # set_steering:
    #   sets steering angle (without drift) limited by max_steering_angle
    #   no noise is taken in consideration
    #   returns the steering set

    def set_steering(self, steering):
        if steering > self.max_steering_angle:
            #print('positiv steering')
            self.steering = self.max_steering_angle
        elif steering < -self.max_steering_angle:
            self.steering = -self.max_steering_angle
            #print('negativ steering')
        else:
            self.steering = steering
        return self.steering

    # --------
    # move:
    #   moves the robot
    #	steering = front wheel steering angle, limited by max_steering_angle
    #	distance = total distance driven, most be non-negative

    def move(self, steering, distance, tolerance = 0.001):
        steering = self.set_steering(steering)
        if distance < 0.0: # cannot go back
            distance = 0.0

        # apply noise
        steering = random.gauss(steering, self.steering_noise)
        distance = random.gauss(distance, self.distance_noise)
        steering = self.set_steering(steering)
        #print ('steering after: ', steering)
        # apply drift
        steering += self.steering_drift

        res = Robot()
        # execute motion:
        turn = tan(steering) * distance / self.length
        #print('turn: ', turn)
        if abs(turn) < tolerance:
            # approximate by straight line motion

            res.x = self.x + (distance * cos(self.orientation))
            res.y = self.y + (distance * sin(self.orientation))
            res.orientation = (self.orientation + turn)

            self.set(res.x,res.y ,res.orientation)
            
            
        else:
            # approximate bicycle model for motion
            radius = distance / turn
            cx = self.x - (sin(self.orientation) * radius)
            cy = self.y + (cos(self.orientation) * radius)

            res.orientation = (self.orientation + turn)
            res.x = cx + (sin(res.orientation) * radius)
            res.y = cy - (cos(res.orientation) * radius)

            self.set(res.x, res.y, res.orientation)
            
        return res

    # --------
    # sense: 
    #    

    def sense2(self):

        return [random.gauss(self.x, self.measurement_noise),
                random.gauss(self.y, self.measurement_noise)]

    # --------
    #
    # sense: returns x- and y- distances to landmarks within visibility range
    #        because not all landmarks may be in this range, the list of measurements
    #        is of variable length. Set measurement_range to -1 if you want all
    #        landmarks to be visible at all times
    #

    def sense(self, landmarks):
        Z = []
        for i in range(len(landmarks)):
            #print("landmark: " , landmarks[i])
            #print("self x y: ", self.x, self.y)
            dx = landmarks[i][0] - self.x #+ self.rand() * self.measurement_noise
            dy = landmarks[i][1] - self.y #+ self.rand() * self.measurement_noise
            #print("dx dy: ", dx, dy)
            if self.measurement_range < 0.0 or abs(dx) + abs(dy) <= self.measurement_range:
                Z.append([i, dx, dy])
        return Z


    # --------
    # measurement_prob
    #    computes the probability of a measurement
    # 

    def measurement_prob(self, measurement):

        # compute errors
        error_x = measurement[0] - self.x
        error_y = measurement[1] - self.y

        # calculate Gaussian
        error = exp(- (error_x ** 2) / (self.measurement_noise ** 2) / 2.0) \
            / sqrt(2.0 * pi * (self.measurement_noise ** 2))
        error *= exp(- (error_y ** 2) / (self.measurement_noise ** 2) / 2.0) \
            / sqrt(2.0 * pi * (self.measurement_noise ** 2))

        return error


    def __repr__(self):
        return '[x=%.5f y=%.5f orient=%.5f]'  % (self.x, self.y, self.orientation)

# ------------------------------------------------
#
# World class:
#   application model
#

class World:
    # --------
    # init:
    #   creates world with track and a robot in it
    #
    
    def __init__(self, radius = 25.0, speed = 0.1, d_t = 1.0,
                 tau_p = 2.0, tau_d = 6.0, tau_i = 0.0,
                 twiddle = False, twiddle_skip_iterations = 100, twiddle_iterations = 200,
                 robot_color = "red", wheel_color = "black"):
        self.radius = radius
        self.speed = speed
        self.d_t = d_t # time interval between steps
        self.robot = Robot(color = robot_color, wheel_color = wheel_color)
        # PID parameters
        self.tau_p = tau_p # proportional CTE factor
        self.tau_d = tau_d # differential CTE factor
        self.tau_i = tau_i # integral CTE factor

        self.pfilter = []
        
        self.twiddle = twiddle # should use twiddle?
        if twiddle_skip_iterations >= twiddle_iterations:
            twiddle_skip_iterations = 0
        self.twiddle_skip_iterations = twiddle_skip_iterations # how many iterations to skip in simulation for twiddle?
        self.twiddle_iterations = twiddle_iterations           # how many iterations to do in simulation for twiddle?
        self.last_cte = 0.0 # last crosstrack error (needed for calculation of differential CTE in PID)
        self.sum_cte = 0.0  # sum of all crosstrack errors so far (needed for calculation of integral CTE in PID)


        ### maze init and spath stuff
        self.mazesize = 10
        self.grid = []
        self.block_size = 1
        self.landmarks = []
        self.goal = []
        self.init_maze()
        self.init = [1, 0]
        #self.goal_x = 0
        #self.goal_y = 0
        self.spath = []
        self.spath_index = 0
        self.sensed_landmarks = []
        

    # --------
    # set_pid_parameters:
    #   sets PID parameters, i.e. factors for proportional, differential and integral crosstrack error 
    
    def set_pid_parameters(self, tau_p, tau_d, tau_i):
        self.tau_p = tau_p
        self.tau_d = tau_d
        self.tau_i = tau_i

    # --------
    # reset_robot:
    #   resets robot without recalculating PID parameters
    
    def reset_robot(self):
        #self.last_cte = self.cte()
        #self.sum_cte = 0.0
        return
        
    # --------
    # restart_robot:
    #   resets robot and recalculates PID parameters
    #   should be called when robot is relocated 

    def restart_robot(self):
        self.reset_robot()
        if self.twiddle:
            print "Twiddling..."
            self.do_twiddle()
            print "Done."

    # --------
    # steer_and_move_robot:
    #   calculates the steering and moves the robot to next location

    def steer_and_move_robot(self):
        d_t = self.d_t
        old_cte = self.last_cte;
        cte = self.cte() # crosstrack error
        d_cte = cte - old_cte
        self.sum_cte += cte
        self.last_cte = cte
        # print "steer and move\n"
        # PID = proportional, differential, integral
        # P (proportional): -tau_p * cte         - steers towards desired path, oscillates
        # D (differential): -tau_d * d_cte / d_t - eliminates oscilations
        # I (integral):     -tau_i * sum_cte     - solves the systematic bias (e.g. wheels are not properly adjusted and are initially pointing to the left)
        steering = - self.tau_p * cte \
                   - self.tau_d * d_cte / d_t \
                   - self.tau_i * self.sum_cte
        
        #steering = 0
        #if abs(cte) > abs(old_cte):
        #    steering = steering * -1
            
        #print ('steering: ', steering)
        self.robot.move(steering, self.speed)

        self.pfilter.move(steering, self.speed)

        self.sensed_landmarks = self.robot.sense(self.landmarks)
        #print self.sensed_landmarks
        #self.pfilter.sense(Z)

    # --------
    # cte:
    #   calculates crosstrack error as a difference between the correct location and the robot's current location


    def cte(self):
        index = self.spath_index
        spath = self.spath
        robot = self.robot
        cte = 0.0
        estimate = self.pfilter.get_position()
        #print("estimate: " , estimate)
        # some basic vector calculations
        dx = spath[index+1][0] - spath[index][0]
        dy = spath[index+1][1] - spath[index][1]
        drx = estimate[0] - spath[index][0]
        dry = estimate[1] - spath[index][1]
        #drx = robot.x - spath[index][0]
        #dry = robot.y - spath[index][1]
        #print("spath[index][0], spath[index][1]: ", spath[index][0], spath[index][1])
        #print("spath[index+1][0], spath[index+1][1]: ", spath[index+1][0], spath[index+1][1])
        #print("dx, dy, drx, dry: ", dx, dy, drx, dry)


        # u is the robot estimate projectes onto the path segment
        u = (drx * dx + dry * dy) / (dx * dx + dy * dy)
        #print("u: ", u)
        # the cte is the estimate projected onto the normal of the path segment
        cte = (dry * dx - drx * dy) / (dx * dx + dy * dy)
        #print('cte: ', cte)


        # pick the next path segment
        if u > 1:
            self.spath_index += 1
            #print ("self.spath_index: ", self.spath_index)
        #else:
        #   return cte
        
        return cte



    # --------
    # init_maze:
    #   init a random maze

    def init_maze(self):
        ### make a more or less random grid
        ###
        mazesize = self.mazesize
        #self.block_size = 11
        self.block_size = (1000 / mazesize)
        self.block_size = (1000 - self.block_size) / mazesize + 1
        
        for m in maze(mazesize,mazesize, False):
            pass
        self.grid = m.tolist()
        self.goal = ([len(self.grid)-2, len(self.grid[0])-1])
        #self.goal = len(self.grid)-2
        #self.goal = len(self.grid[0])-1
        #self.goal = [4,4]
        #print("self.goal_x: ", self.goal_x)
        #print("self.goal_y: ", self.goal_y)
        self.landmarks = self.list_of_landmarks()
        #print ("landmarks: ", self.landmarks)


    def list_of_landmarks(self):
        l = []
        grid = self.grid
        #print self.grid
        first = True
        last_row = len(grid) - 1
        # append the start and stop landmarks
        #l.append([1 * self.block_size,0])
        #l.append([2 * self.block_size,0])
        l.append([0, 1 * self.block_size])
        l.append([0, 2 * self.block_size])
        
        for y in range(len(grid)):
            last_in_row = len(grid[0]) - 1
            for x in range(len(grid[0])):
                if grid[y][x] != 0:
                    # first row, will be not evaluated
                    if y == 0:
                        continue
                    #if y >= last_row:
                    #    continue
                    if grid[y][x - 1] == 0 and grid[y - 1][x - 1] == 1:
                        #l.append([y * self.block_size,x * self.block_size])
                        l.append([x * self.block_size, y * self.block_size])
                    if grid[y][x - 1] == 0 and grid[y + 1][x] == 0:
                        #l.append([y * self.block_size + self.block_size,x * self.block_size])
                        l.append([x * self.block_size, y * self.block_size + self.block_size])
                    if grid[y][x - 1] == 0 and grid[y - 1][x] == 0:
                        #l.append([y * self.block_size,x * self.block_size])
                        l.append([x * self.block_size, y * self.block_size])
                    if grid[y][x - 1] == 1 and grid[y - 1][x] == 1 and x != 0:
                        #l.append([y * self.block_size,x * self.block_size])
                        l.append([x * self.block_size, y * self.block_size])
                    
                    

                    if x < last_in_row:
                        if grid[y][x + 1] == 0 and grid[y - 1][x + 1] == 1:
                            #l.append([y * self.block_size,x  * self.block_size + self.block_size])
                            l.append([x  * self.block_size + self.block_size, y * self.block_size])
                        if grid[y][x + 1] == 0 and grid[y + 1][x] == 0:
                            #l.append([y * self.block_size + self.block_size,x * self.block_size + self.block_size])
                            l.append([x * self.block_size + self.block_size, y * self.block_size + self.block_size])
                        if grid[y][x + 1] == 0 and grid[y - 1][x] == 0:
                            #l.append([y * self.block_size, x * self.block_size + self.block_size])
                            l.append([ x * self.block_size + self.block_size, y * self.block_size])

                        if grid[y][x + 1] == 1 and grid[y - 1][x] == 1:
                            #l.append([y * self.block_size, x * self.block_size + self.block_size])
                            l.append([x * self.block_size + self.block_size, y * self.block_size])
                    
                        

        #l.append([self.goal[0] * self.block_size,self.goal[1] * self.block_size + self.block_size])
        #l.append([self.goal[0] * self.block_size + self.block_size,self.goal[1] * self.block_size + self.block_size])
        l.append([self.goal[1] * self.block_size + self.block_size, self.goal[0] * self.block_size])
        l.append([self.goal[1] * self.block_size + self.block_size, self.goal[0] * self.block_size + self.block_size])
        return l


    # --------
    # getAPlan:
    #   get the A* search result
    
    def getAPlan(self):
        #print("goal for path: ", self.goal)
        ##goal = [self.goal_x, self.goal_y]
        path = plan(self.grid, self.init, self.goal)
        path.astar()
        path.smooth(0.1, 0.05)
        #print(path.spath)
        spath = path.spath

        # convert the spath to the world, with the block_size
        converted_path = []
        block_size = self.block_size
        for i in range(len(spath)):
            converted_path.append([spath[i][1] * block_size + (block_size / 2), spath[i][0] * block_size + (block_size / 2)])
        self.spath = converted_path
        #print(converted_path)
    
    # --------
    # simulate_for_twiddle:
    #   simulates robot motion with given PID parameters through the given number of iterations;
    #   returns the error accumulated after treshold (treshold is there to give robot a chance to stabilize it's motion)
    
    def simulate_for_twiddle(self, tau_p, tau_d, tau_i):
        self.reset_robot()
        # save original robot position
        saved_robot = self.robot.copy()
        
        saved_tau_p = self.tau_p
        saved_tau_d = self.tau_d
        saved_tau_i = self.tau_i
        self.set_pid_parameters(tau_p, tau_d, tau_i)
        
        error = 0
        for i in range(self.twiddle_iterations):
            self.steer_and_move_robot()
            if i > self.twiddle_skip_iterations:
                error += self.cte()**2

        # restore PID parameters
        self.set_pid_parameters(saved_tau_p, saved_tau_d, saved_tau_i)
        
        # restore robot
        self.robot = saved_robot
        
        return error
        
    # --------
    # do_twiddle:
    #   uses twiddling to try to find the best PID parameters;
    #   starts from some set of parameter values and keeps adjusting them until the simulation produces lowest error
    
    def do_twiddle(self, tolerance = 0.1):
        if self.twiddle:
            p = [0, 0, 0]     # initial parameters
            dp = [1., 1., 1.] # potential changes
            
            best_error = self.simulate_for_twiddle(p[0], p[1], p[2])
            cnt = 0 # iteration counter, just for info
            while sum(dp) > tolerance:
                cnt += 1
                for i in range(len(p)): # go through all the parameters
                    original_p_i = p[i]
                    p[i] = original_p_i + dp[i]
                    error = self.simulate_for_twiddle(p[0], p[1], p[2])
                    if error < best_error:
                        best_error = error
                        dp[i] *= 1.1
                    else:
                        # we got worse error, try lowering parameter instead:
                        p[i] = original_p_i - dp[i]
                        error = self.simulate_for_twiddle(p[0], p[1], p[2])
                        if error < best_error:
                            best_error = error
                            dp[i] *= 1.1  # increase this parameter change step
                        else: # both attempts failed
                            p[i] = original_p_i
                            dp[i] *= 0.9 # decrease this parameter change step
            print p
            self.set_pid_parameters(p[0], p[1], p[2])

# ------------------------------------------------
#
# Window class:
#   application view
#

class Window:
    
    # --------
    # init:
    #   creates window on screen
    
    def __init__(self, width = 500, height = 500, title = ""):
        self.top = Tk()
        self.canvas = Canvas(self.top, bg="white", height=height, width=width)
        self.canvas.pack(fill=BOTH, expand=1)
        self.top.wm_attributes("-topmost", 1)
        self.top.resizable(FALSE,FALSE)
        #self.place_to_center()
        self.top.title(title)

    # --------
    # place_to_center:
    #   places window to the center of the screen
    
    def place_to_center(self):
        ws = self.top.winfo_screenwidth()
        hs = self.top.winfo_screenheight()
        self.top.update_idletasks()
        w=self.top.winfo_reqwidth()
        h=self.top.winfo_reqheight()
        x=ws/2-w/2 
        y=hs/2-h/2
        self.top.geometry('%dx%d+%d+%d' % (w, h, x, y))

# ------------------------------------------------
#
# Utils2D class:
#   utilities for 2D calculations and transformations
#

class Utils2D:
    
    # --------
    # rotate:
    #   returns point rotated around base_point for given angle
    
    @staticmethod
    def rotate(point, base_point, angle):
        x = point[0]
        y = point[1]
        # translate base point to 0, 0
        p = (x-base_point[0], y-base_point[1])
        # rotate around origin:
        x = p[0]*cos(angle) - p[1]*sin(angle)
        y = p[0]*sin(angle) + p[1]*cos(angle)
        # translate back to base point
        p = (x + base_point[0], y + base_point[1])
        return p
    
    # --------
    # get_angle:
    #   returns orientation of the vector defined by from_point and to_point 

    @staticmethod
    def get_angle(from_point, to_point):
        from_x = from_point[0]
        from_y = from_point[1]
        to_x = to_point[0]
        to_y = to_point[1]
        return atan2(to_y-from_y,to_x-from_x)
    
# ------------------------------------------------
#
# RacetrackAnimation class:
#   application controller
#

class CarAnimation:
    
    # --------
    # init:
    #   prepares model by creating world, and view by creating window, defining mapping between world and screen coordinates, placing origin etc.
    
    def __init__(self,
                 win_width = 1000, win_height = 1000, zoom_out = 1,
                 radius = 25.0,
                 twiddle = False, twiddle_skip_iterations = 100, twiddle_iterations = 200,
                 draw_trail = False, trail_length = 200, trail_color="magenta",
                 tau_p = 4.0, tau_d = 15.0, tau_i = 0.0,
                 path_color="blue", robot_color="red", wheel_color="black"):
        # win_width, win_height - window dimensions in screen coordinates
        
        self.world = World(radius,
                           twiddle = twiddle,
                           twiddle_skip_iterations = twiddle_skip_iterations,
                           twiddle_iterations = twiddle_iterations,
                           robot_color = robot_color, wheel_color = wheel_color,
                           tau_p = tau_p, tau_d = tau_d, tau_i = tau_i)

        # should the trail be kept and drawn?
        self.draw_trail = draw_trail
        self.trail_color = trail_color
        self.trail_length = trail_length
        self.trail = []
        
        # stage dimensions in world coordinates:
        stage_width  = 5.0 * radius * zoom_out # track is 4*radius wide + margin radius/2 on each side
        stage_height = 5.0 * radius * zoom_out # track is 2*radius high + margin radius/2 on each side
        #stage_width  = 200
        #stage_height = 200

        # scale factor (screen = world * scale)
        self.x_scale = float(win_width) / stage_width
        self.y_scale = float(win_height) / stage_height
        
        self.window = Window(width = win_width,
                             height = win_height,
                             title = 'Animated Car (by mmyself)')
        top = self.window.top

        # origin in screen coordinates:
        self.origin = (win_width/2. - 2 * radius * self.x_scale,
                       win_height/2. + radius * self.y_scale)

        self.path_color = path_color

        # states:
        self.initializing = False
        self.dragging = False
        self.running = False

        self.iterate_id = None

        
        
    # --------
    # to_screen:
    #   returns screen coordinates of a point given in world coordinates
    
    def to_screen(self, world_point):
        x = self.origin[0] + world_point[0] * self.x_scale
        y = self.origin[1] - world_point[1] * self.y_scale
        return (x, y)

    # --------
    # to_world:
    #   returns world coordinates of a point given in screen coordinates
    
    def to_world(self, screen_point):
        x = (screen_point[0] - self.origin[0]) / self.x_scale
        y = -(screen_point[1] - self.origin[1]) / self.y_scale
        return (x, y)


    
        

    # --------
    # draw_maze:
    #   draw the given maze

    def draw_maze(self):
        canvas = self.window.canvas
        world = self.world
        block_size = world.block_size
        grid = world.grid
        #print self.grid
        for y in range(len(grid)):
            for x in range(len(grid[0])):
                if grid [y][x] != 0:
                    new = [x * block_size,y * block_size]
                    new2 = [x * block_size + block_size,y * block_size + block_size]
                    canvas.create_rectangle(new[0], new[1], new2[0], new2[1], fill="blue")
                    
    
        
    # --------
    # draw_spath:
    #   draws spath on canvas

    def draw_spath(self):
        canvas = self.window.canvas
        spath = self.world.spath
        block_size = self.world.block_size
        #print(self.spath)
        for i in range(len(spath)):
            if i == 0:
                   continue
            prev = [spath[i-1][1], spath[i-1][0]]
            this = [spath[i][1], spath[i][0]]
            canvas.create_line(prev[1], prev[0],
                                  this[1], this[0],
                                   fill = "red")

    # ----------
    # draw_landmarks
    #

    def draw_landmarks(self):
        canvas = self.window.canvas
        landmarks = self.world.landmarks
        for i in range(len(landmarks)):
            canvas.create_oval(landmarks[i][0] - 3, landmarks[i][1] - 3 , landmarks[i][0] + 3, landmarks[i][1] + 3, width=2, outline='red', fill='red')
    

    # ----------
    # draw_sensed_landmarks
    #

    def draw_sensed_landmarks(self):
        canvas = self.window.canvas
        landmarks = self.world.sensed_landmarks
        robot = self.world.robot
        robot_x = robot.x
        robot_y = robot.y
        
        for i in range(len(landmarks)):
            #print("x,y: ", robot_x,robot_y)
            #print("sensed landmark: ", landmarks[i][1], landmarks[i][2])
            canvas.create_oval(robot_x + landmarks[i][1] - 3, robot_y + landmarks[i][2] - 3 ,robot_x + landmarks[i][1] + 3, robot_y + landmarks[i][2] + 3, width=1, outline='green', fill='green')
            #canvas.create_oval(robot_x  - 3, robot_y  - 3 ,robot_x  + 3, robot_y + 3, width=1, outline='green', fill='green')

    # --------
    # draw_path:
    #   draws path on canvas
    
    def draw_path(self):
        radius = self.world.radius
        color = self.path_color
        canvas = self.window.canvas
        
        from_point = self.to_screen((radius, 2*radius))
        to_point = self.to_screen((3*radius, 2*radius))
        canvas.create_line(from_point[0], from_point[1], to_point[0], to_point[1], fill=color)
        from_point = self.to_screen((radius, 0))
        to_point = self.to_screen((3*radius, 0))
        canvas.create_line(from_point[0], from_point[1], to_point[0], to_point[1], fill=color)
        from_point = self.to_screen((0.0, 2*radius))
        to_point = self.to_screen((2*radius, 0.0))
        canvas.create_arc(from_point[0], from_point[1], to_point[0], to_point[1], start=90, extent=180, style="arc", outline=color)
        from_point = self.to_screen((2*radius, 2*radius))
        to_point = self.to_screen((4*radius, 0.0))
        canvas.create_arc(from_point[0], from_point[1], to_point[0], to_point[1], start=-90, extent=180, style="arc", outline=color)

    # --------
    # draw_robot:
    #   draws the robot with it's current position and orientation 

    def draw_robot(self):
        robot = self.world.robot
        x = robot.x
        y = robot.y
        orientation = robot.orientation
        length = robot.length
        width = robot.width
        wheel_length = robot.wheel_length
        wheel_width = robot.wheel_width
        canvas = self.window.canvas
        steering_drift = robot.steering_drift
        steering = robot.steering
        color = robot.color
        wheel_color = robot.wheel_color
        
        # get four points of a car directing east
        # (x,y position is position in the middle between rear wheels)
        corners = [(x-wheel_length, y+width/2+wheel_length/2), # back left
                   (x-wheel_length, y-width/2-wheel_length/2), # back right
                   (x+length+wheel_length, y-width/2-wheel_length/2), # front right
                   (x+length+wheel_length, y+width/2+wheel_length/2)] # front left
        wheel_centers = [(x, y+width/2), # back left
                         (x, y-width/2), # back right
                         (x+length, y-width/2), # front right
                         (x+length, y+width/2)] # front left
        wheels = []
        for i in range(len(wheel_centers)):
            xc = wheel_centers[i][0]
            yc = wheel_centers[i][1]
            wheel_corners = [(xc-wheel_length/2, yc+wheel_width/2), # back left
                             (xc-wheel_length/2, yc-wheel_width/2), # back right
                             (xc+wheel_length/2, yc-wheel_width/2), # front right
                             (xc+wheel_length/2, yc+wheel_width/2)] # front left
            wheels.append(wheel_corners)
        # rotate for the orientation
        n = len(corners)
        for i in range(n):
            #corners[i] = self.to_screen(Utils2D.rotate(corners[i], (x, y), orientation))
            corners[i] = Utils2D.rotate(corners[i], (x, y), orientation)
        canvas.create_polygon(corners[0][0], corners[0][1], \
                              corners[1][0], corners[1][1], \
                              corners[2][0], corners[2][1], \
                              corners[3][0], corners[3][1], \
                              outline=color, fill="")

        m = len(wheels) # number of wheels
        n = len(wheels[0]) # number of corners
        for k in range(m):
            corners = wheels[k]
            if k >= m-2:
                wheel_x = wheel_centers[k][0]
                wheel_y = wheel_centers[k][1]
                for i in range(n):
                    corners[i] = Utils2D.rotate(corners[i], (wheel_x, wheel_y), steering+steering_drift)
            for i in range(n):
                #corners[i] = self.to_screen(Utils2D.rotate(corners[i], (x, y), orientation))
                corners[i] = Utils2D.rotate(corners[i], (x, y), orientation)
            canvas.create_polygon(corners[0][0], corners[0][1], \
                                  corners[1][0], corners[1][1], \
                                  corners[2][0], corners[2][1], \
                                  corners[3][0], corners[3][1], \
                                  fill=wheel_color)

        if self.draw_trail:
            #self.trail.append(self.to_screen((robot.x, robot.y)))
            self.trail.append((robot.x, robot.y))
            if len(self.trail) > self.trail_length:
                self.trail.pop(0)
            for i in range(len(self.trail)):
                if i == 0:
                    continue
                prev_point = self.trail[i-1]
                this_point = self.trail[i]
                canvas.create_line(prev_point[0], prev_point[1],
                                   this_point[0], this_point[1],
                                   fill = self.trail_color)


    

    # --------
    # mousePressed:
    #   mouse press event handler;
    #   stops animation and relocates robot to the click location

    def mousePressed(self, event):
        
        if self.running:
            self.running = False
            self.window.top.after_cancel(self.iterate_id)
            self.initializing = False
        else:
            self.initializing = True
        #if self.running:
        #    self.running = False
        #    self.window.top.after_cancel(self.iterate_id)
        #    self.initializing = True
        #if self.initializing:
        #    self.dragging = True
        #    self.trail = []
        #   point = self.to_world((event.x, event.y))
        #    self.start_drag_x = point[0]
        #    self.start_drag_y = point[1]
        #    robot = self.world.robot
        #    robot.set(self.start_drag_x, self.start_drag_y, robot.orientation)
        #    self.redraw()
            
    # --------
    # mouseDragged:
    #   mouse dragg event handler;
    #   rotates robot per mouse position
    
    #def mouseDragged(self, event):
    #    if self.dragging:
    #        point = self.to_world((event.x, event.y))
    #        new_x = point[0]
    #        new_y = point[1]
    #        new_orientation = Utils2D.get_angle(
    #            (self.start_drag_x, self.start_drag_y),
    #            (new_x, new_y))
    #        robot = self.world.robot
    #        robot.set(robot.x, robot.y, new_orientation)
    #        self.redraw()
        
    # --------
    # mouseReleased:
    #   mouse release event handler;
    #   resumes animation with the robot starting from the new position
    
    def mouseReleased(self, event):
        if self.initializing:
            self.world.getAPlan();
            self.initializing = False
            self.iterate_id = self.window.top.after(50, self.start_iterations)
        
    # --------
    # iterate:
    #   periodically calculates and redraws new robot position

    def iterate(self):
        self.running = True
        world = self.world
        world.steer_and_move_robot();
        self.redraw()
        self.iterate_id = self.window.top.after(50, self.iterate)
        
    # --------
    # start_iterations:
    #   start animation
    
    def start_iterations(self):
        if self.running:
            self.window.top.after_cancel(self.iterate_id)  
        self.initializing = False
        self.dragging = False
        self.running = True
        self.world.restart_robot()
        self.iterate()
            
    # --------
    # toggle_pause:
    #   toggles animation on/off
    
    def toggle_pause(self):
        if not self.initializing:
            if not self.running:
                self.running = True
                self.iterate()
            else:
                self.window.top.after_cancel(self.iterate_id)
                self.running = False
            
    # --------
    # redraw:
    #   redraws world

    def redraw(self):
        canvas = self.window.canvas
        canvas.delete(ALL)
        self.draw_maze()
        self.draw_spath()
        self.draw_robot()
        self.draw_landmarks()
        self.draw_sensed_landmarks()

    # --------
    # run:
    #   initially positions the robot, binds keyboard/mouse controls and draws the world 
    
    def run(self, robot_x = 0.0, robot_y = None, robot_orientation = pi/2.0, speed=2.0):
        # default argument values
        if robot_y == None:
            robot_y = self.world.radius
        
        top = self.window.top
        robot = self.world.robot
        robot_x = self.world.block_size / 2
        robot_y = self.world.block_size  + (self.world.block_size / 2)
        
        #print("robot_x, robot_y: ", robot_x, robot_y)
        #robot_x = 100
        #robot_y = 100
        robot.set(robot_x, robot_y, robot_orientation)
        #print("robot.steering_noise, robot.distance_noise, robot.measurement_noise: ", robot.steering_noise, robot.distance_noise, robot.measurement_noise)
        self.world.pfilter = particles(robot_x, robot_y, robot_orientation,
                           robot.steering_noise, robot.distance_noise, robot.measurement_noise)
        
        self.world.speed = speed
        top.bind('<Key-Escape>',lambda e: e.widget.destroy())
        top.bind('<Return>',lambda e: self.start_iterations())
        top.bind('<space>',lambda e: self.toggle_pause())
        top.bind('<Button-1>', lambda event: self.mousePressed(event))
        top.bind('<B1-Motion>', lambda event: self.mouseDragged(event))
        top.bind('<ButtonRelease-1>', lambda event: self.mouseReleased(event))
        self.initializing = True
        self.world.reset_robot()
        self.redraw()
        top.mainloop()     

# ------------------------------------------------
#
# main:
#   application entry point
#

#animation = RacetrackAnimation(radius = 50, tau_p = 10.0, tau_d = 15.0, tau_i = 0.0)
animation = CarAnimation(radius = 50, tau_p = 2.0, tau_d = 20.0, tau_i = 0.0, draw_trail = True)
#animation = RacetrackAnimation(radius = 50, tau_p = 2.0, tau_d = 20.0, tau_i = 0.0, draw_trail = True, zoom_out = 2)
#animation = RacetrackAnimation(radius = 50, twiddle = True, draw_trail = True)
#animation = RacetrackAnimation(radius = 50, twiddle = True, twiddle_skip_iterations = 20, twiddle_iterations = 400, draw_trail = True, zoom_out = 2)
#animation = RacetrackAnimation(radius = 50, twiddle = True, twiddle_skip_iterations = 20, twiddle_iterations = 400, draw_trail = True)
animation.run(robot_x = 0.0, robot_orientation = 0)
#animation.run(robot_x = animation.world.radius,
#              robot_y = 2.2 * animation.world.radius,
#              robot_orientation = 0.0)
