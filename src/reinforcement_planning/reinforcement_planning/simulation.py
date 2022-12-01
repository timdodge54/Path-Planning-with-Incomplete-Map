import math
from platform import node
from queue import PriorityQueue

import numpy
import numpy as np
import time
import random
from Agent import Agent
import torch as T

import matplotlib.pyplot as plt

show_animation = True


class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
                                                                     open_set[
                                                                         o]))
            current = open_set[c_id]

            # show graph
            # if show_animation:  # pragma: no cover
            #     plt.plot(self.calc_grid_position(current.x, self.min_x),
            #              self.calc_grid_position(current.y, self.min_y), "xc")
            #     # for stopping simulation with the esc key.
            #     plt.gcf().canvas.mpl_connect('key_release_event',
            #                                  lambda event: [exit(
            #                                      0) if event.key == 'escape' else None])
            #     if len(closed_set.keys()) % 10 == 0:
            #         plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry 

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        # print("min_x:", self.min_x)
        # print("min_y:", self.min_y)
        # print("max_x:", self.max_x)
        # print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        # print("x_width:", self.x_width)
        # print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion


class Simulation:
    def __init__(self, robot_radius, grid_size, obstacle_count, sx, sy, gx, gy):
        self.robot_radius = robot_radius
        self.grid_size = grid_size
        self.obstacle_count = obstacle_count
        self.gx = gx
        self.gy = gy
        self.sx = sx
        self.sy = sy
        self.tx = [sx]
        self.ty = [sy]
        self.ix = [sx]
        self.iy = [sy]
        self.pnt2 = 1
        self.theta = [math.pi / 2]
        self.dist_tolerance = robot_radius
        self.pnt = 0
        self.steps = 0
        self.getMap()
        angle = np.linspace( 0 , 2 * np.pi , 150 )
        radius = self.robot_radius - 0.5
        xpoints = sx + radius * np.cos( angle )
        ypoints = sy + radius * np.sin( angle )
        self.robot, = plt.plot(xpoints, ypoints, marker = ".", color = 'r')
        self.robotAngle, = plt.plot(sx + radius * np.cos( self.theta[-1] ), sy + radius * np.sin( self.theta[-1]  ), marker = ".", color = 'b')

    def __motion(
        self, v_left: float, v_right: float, x: float, y: float, theta: float, dt: float
    ):
        """Calculate the forward kinematics of the robot"""
        aveVel = (1 / 2) * (v_right + v_left)
        x_Dot = -aveVel * math.sin(theta - math.pi / 2)
        y_Dot = aveVel * math.cos(theta - math.pi / 2)
        theta_Dot = (v_right - v_left) / (self.robot_radius*2)

        x_new = x + x_Dot * dt
        y_new = y + y_Dot * dt
        theta_new = (theta + theta_Dot * dt) % (2*math.pi)

        return x_new, y_new, theta_new

    def getMap(self):
        map = np.zeros((71, 71), dtype=bool)

        while True:
            # set obstacle positions
            ox, oy = [], []
            #border
            for i in range(-10, 60):
                ox.append(i)
                oy.append(-10)
                map[i + 10, 0] = True
            for i in range(-10, 60):
                ox.append(60)
                oy.append(i)
                map[70, i + 10] = True
            for i in range(-10, 61):
                ox.append(i)
                oy.append(60)
                map[i + 10, 70] = True
            for i in range(-10, 61):
                ox.append(-10)
                oy.append(i)
                map[0, i + 10] = True

            #obstacles
            for i in range(self.obstacle_count):
                x = random.randint(-9, 60)
                y = random.randint(-9, 60)
                ox.append(x)
                oy.append(y)
                map[x + 10, y + 10] = True

            self.ox = ox
            self.oy = oy
            self.map = map

            a_star = AStarPlanner(ox, oy, self.grid_size, self.robot_radius)
            rx, ry = a_star.planning(self.sx, self.sy, self.gx, self.gy)
            if len(rx) > 1:
                break

        rx.reverse()
        ry.reverse()
        self.rx = rx
        self.ry = ry

    def getPath(self):
        # negative if left of path 
        minVal = self.distance(self.tx[-1], self.ty[-1], self.rx[0], self.ry[0])
        self.pnt = 0
        for i in range(1, len(self.rx)):
            new = self.distance(self.tx[-1], self.ty[-1], self.rx[i], self.ry[i])
            if new < minVal:
                self.pnt = i
                minVal = new
        vector_to_path = (self.rx[self.pnt] - self.tx[-1], self.ry[self.pnt] - self.ty[-1])
        # print(f'path distance: {pathDistance}')
        return vector_to_path[0], vector_to_path[1]

    def getGoal(self): 
        goal_vector = (self.gx - self.rx[self.pnt], self.gy - self.ry[self.pnt])
        # print(f'goal: {goalDistance}')
        return goal_vector

    def getObstacle(self):
        obsDistanceforward = 100
        obsDistanceLeft = 100
        obsDistanceRight = 100
        for i in range(10):
            if self.map[round(self.tx[-1] + i * math.cos(self.theta[-1])), round(self.ty[-1] + i * math.sin(self.theta[-1]))]:
                obsDistanceforward = self.distance(self.tx[-1], self.ty[-1], self.ix[-1] + round(i * math.cos(self.theta[-1])), self.iy[-1] + round(i * math.sin(self.theta[-1])))
                break
        for i in range(10):
            leftTheta = self.theta[-1] - (60 * math.pi / 180) 
            if self.map[round(self.tx[-1] + i * math.cos(leftTheta)), round(self.ty[-1] + i * math.sin(leftTheta))]:
                obsDistanceLeft = self.distance(self.tx[-1], self.ty[-1], self.ix[-1] + round(i * math.cos(leftTheta)), self.iy[-1] + round(i * math.sin(leftTheta)))
                break
        for i in range(10):
            rightTheta = self.theta[-1] + (60 * math.pi / 180) 
            if self.map[round(self.tx[-1] + i * math.cos(rightTheta)), round(self.ty[-1] + i * math.sin(rightTheta))]:
                obsDistanceRight = self.distance(self.tx[-1], self.ty[-1], self.ix[-1] + round(i * math.cos(rightTheta)), self.iy[-1] + round(i * math.sin(rightTheta)))
                break
        # print(f'obstacle Distance: {obsDistance}')
        return (obsDistanceforward, obsDistanceLeft, obsDistanceRight)

    def getTheta(self):
        #angle difference
        pathVector = (self.rx[self.pnt + 1] - self.rx[self.pnt], self.ry[self.pnt + 1] - self.ry[self.pnt])
        robotVector = (math.cos(self.theta[-1]), math.sin(self.theta[-1]))
        distance = self.distance(0, 0, pathVector[0], pathVector[1]) * self.distance(0, 0, robotVector[0], robotVector[1])

        dot = pathVector[0] * robotVector[0] + pathVector[1] * robotVector[1]
        det = pathVector[0] * robotVector[1] - pathVector[1] * robotVector[0]

        #positive is convergent
        #negative is divergent

        # left = np.matrix([[self.tx[-1] - self.rx[self.pnt]], [self.ty[-1] - self.ry[self.pnt]]])
        # matrix = np.matrix([[-robotVector[0], pathVector[0]], [-robotVector[1], pathVector[1]]])
        # try:
        #    inverse = np.linalg.inv(matrix)
        #    solution = numpy.matmul(inverse,left)
        #    if solution[0] < 0 or solution[1] < 0:
        #        deltTheta *= -1
        # except:
        #    deltTheta = 0
        #print(f'Delta Theta: {deltTheta}')
        return math.atan2(det,dot)
    
    def getReward(self):
        x, y = self.getPath()
        gx, gy = self.getGoal()
        path_distance = numpy.sqrt(x*x + y*y)
        goal_distance = numpy.sqrt(gx*gx + gy * gy)

        win = 100 if self.distance(self.tx[-1], self.ty[-1], self.gx, self.gy) < self.dist_tolerance else 0
        obstacles = self.getObstacle()
        return 0.1*obstacles[0] - path_distance - goal_distance + self.getTheta() * self.robot_radius - 1 + win
    
    def isDone(self):
        term = False
        if self.distance(self.tx[-1], self.ty[-1], self.gx, self.gy) < self.dist_tolerance:
            term = True
        if self.distance(self.tx[-1], self.ty[-1], self.rx[self.pnt], self.ry[self.pnt]) > 5 * self.robot_radius:
            term = True
        obstacles = self.getObstacle()
        if obstacles[0] <= self.dist_tolerance or obstacles[1] <= self.dist_tolerance or obstacles[2] <= self.dist_tolerance:
            term = True
        return term

    def step(self, action):
        vLeft, vRight = action[0], action[1]
        dt = .5
        x, y, theta = self.__motion(vLeft, vRight, self.tx[-1], self.ty[-1], self.theta[-1], dt)

        ##replaced by finding the actual new point and going there
        self.tx.append(x)
        self.ty.append(y)
        self.theta.append(theta)
        self.ix.append(round(x))
        self.iy.append(round(y))
        self.pnt2 += 1
        self.steps += 1
        path_x, path_y = self.getPath()
        g_x, g_y = self.getGoal()
        obstacles = self.getObstacle()
        return [path_x, path_y, g_x, g_y, obstacles[0], obstacles[1], obstacles[2], self.getTheta(),  self.tx[-1], self.ty[-1], self.theta[-1]], self.getReward(), self.isDone()

    def print(self, end=True):
        plt.plot(self.ox, self.oy, ".k")
        plt.plot(self.sx, self.sy, "og")
        plt.plot(self.gx, self.gy, "xb")
        plt.plot(self.rx, self.ry, '-r')
        plt.grid(True)
        plt.axis("equal")
        plt.plot(self.tx, self.ty, "-g")        

    def show(self, x, y, theta):
        self.robot.remove()
        self.robotAngle.remove()
        angle = np.linspace( 0 , 2 * np.pi , 150 )
        radius = self.robot_radius - 0.5
        xpoints = x + radius * np.cos( angle )
        ypoints = y + radius * np.sin( angle )
        self.robot, = plt.plot(xpoints, ypoints, marker = ".", color = 'r')
        self.robotAngle, = plt.plot(x + radius * math.cos( theta ), y + radius * math.sin( theta ), marker = ".", color = 'b')

    def showPath(self):
        self.print()
        for i in range(len(self.tx)):
            self.show(self.tx[i], self.ty[i], self.theta[i])
            plt.pause(0.5)
        plt.show()

    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)

    def reset(self):
        self.tx = [self.sx]
        self.ty = [self.sy]
        self.pnt2 = 1
        self.theta = [math.pi / 2]
        self.pnt = 0      
        self.getMap()
        self.steps = 0

        path_x, path_y = self.getPath()
        g_x, g_y = self.getGoal()
        return [path_x, path_y, g_x, g_y, self.getObstacle(), self.getTheta(), self.ix[-1], self.iy[-1], self.theta[-1]]

def main():
    print(__file__ + " start!!")
            # start and goal position
    sx = 10  # [m]
    sy = 10  # [m]
    gx = 50  # [m]
    gy = 50  # [m]
    grid_size = 2  # [m]
    robot_radius = 1.0  # [m]
    obstacle_count = 25

    sim = Simulation(robot_radius, grid_size, obstacle_count, sx, sy, gx, gy)
    for i in range(10):
        sim.step([0.5,0.5])
    sim.showPath()

    # agent = Agent(
    # alpha=0.000025,
    # beta=0.00025,
    # input_dims=[9],
    # tau=0.001,
    # batch_size=64,
    # fc1_dims=400,
    # fc2_dims=300,
    # n_actions=2,
    # action_range=1
    # )

    # agent.load_models()
    # print(T.cuda.is_available())
    # np.random.seed(0)

    # score_history = []
    # for i in range(1000):
    #     done = False
    #     score = 0
    #     obs = sim.reset()
    #     while not done:
    #         act = agent.choose_action(obs)
    #         new_state, reward, done = sim.step(act)
    #         agent.remember(obs, act, reward, new_state, int(done))
    #         agent.learn()
    #         score += reward
    #         obs = new_state


    #     score_history.append(score)
    #     print(
    #         "episode",
    #         i,
    #         "score %.2f" % score,
    #         "100 game average %.2f" % np.mean(score_history[-100:]),
    #     )

    #     if i % 25 == 0:
    #         agent.save_models()




if __name__ == '__main__':
    main()
