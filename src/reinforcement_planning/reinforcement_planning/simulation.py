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
        self.motion = self.get_l()
        self.calc_obstacle_map(ox, oy)
        self.nodes = [[None for x in range(resolution)] for y in range(resolution)]
        self.pq = PriorityQueue(100000)
    
    class Node:
        def __init__(self, x, y, cost, steps, parent_index, isObstacle=False):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.steps = steps
            self.parent_index = parent_index
            self.isObstacle = isObstacle

        def __gt__(self, other):
            return self.cost > other.cost

        def __lt__(self, other):
            return self.cost < other.cost 

        def __eq__(self, other):
            if(other == None):
                return False
            return self.cost == other.cost  

        def __str__(self):
            if(self.isObstacle):
                return str('x')
            return str(f'{self.x},{self.y}, {self.parent_index}|')

    def planning(self, sx, sy, gx, gy, ox, oy):

        
        start_node = self.Node(sx + 10, sy + 10, self.calc_heuristic(gx + 10, gy + 10, sx + 10, sy + 10), 0, (-1,-1))
        current = start_node

        self.nodes[start_node.x][start_node.y] = start_node

        for i in range(len(ox)):
            self.nodes[ox[i] + 10][oy[i]+ 10] = self.Node(ox[i] + 10, oy[i] + 10, 0, 0, (0, 0), True)

        j = 0
        while(current.x != (gx + 10) or current.y != (gy + 10)):
            j += 1

            for i, _ in enumerate(self.motion):
                x = current.x + self.motion[i][0]
                y = current.y + self.motion[i][1]
                step = self.motion[i][2]
                if(self.nodes[x][y] is None): # Visited
                    node = self.Node(x, y, current.steps + self.calc_heuristic(gx, gy, x, y), current.steps + step, (current.x, current.y))
                    self.nodes[node.x][node.y] = node
                    self.pq.put(node)
            
            current = self.pq.get()    
        
        self.nodes[current.x][current.y] = node
        
        # for row in self.nodes:
        #     for col in row:
        #         if col == None:
        #             print('N', end= ' ')
        #         else:
        #             print(col, end=' ')
        #     print()

        return self.calc_final_path(current)

    def calc_final_path(self, goal_node):
        # generate final course
        rx = [goal_node.x - 10]
        ry = [goal_node.y - 10]
        parent_index = goal_node.parent_index
        while parent_index != (-1, -1):
            n = self.nodes[int(parent_index[0])][int(parent_index[1])]
            rx.append(n.x - 10)
            ry.append(n.y - 10)
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(x1, y1, x2, y2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(x2 - x1, y2 - y1)
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

    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        #print("min_x:", self.min_x)
        #print("min_y:", self.min_y)
        #print("max_x:", self.max_x)
        #print("max_y:", self.max_y)

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
    def get_l():
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

    def __motion(
        self, v_left: float, v_right: float, x: float, y: float, theta: float, dt: float
    ):
        """Calculate the forward kinematics of the robot"""
        aveVel = (1 / 2) * (v_right + v_left)
        x_Dot = -aveVel * math.sin(theta)
        y_Dot = aveVel * math.cos(theta)
        theta_Dot = (v_right - v_left) / (self.robot_radius*2)

        x_new = x + x_Dot * dt
        y_new = y + y_Dot * dt
        theta_new = (theta + theta_Dot * dt) % 2*math.pi


        return x_new, y_new, theta_new

    def getMap(self):
        map = np.zeros((71, 71), dtype=bool)

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
        rx, ry = a_star.planning(self.sx, self.sy, self.gx, self.gy, ox, oy)
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
        goalDistance = self.distance(self.rx[self.pnt], self.ry[self.pnt], self.gx, self.gy)
        # print(f'goal: {goalDistance}')
        return goalDistance

    def getObstacle(self):
        obsDistance = 100
        for i in range(10):
            if self.map[round(self.tx[-1] + i * math.cos(self.theta[-1])), round(self.ty[-1] + i * math.sin(self.theta[-1]))]:
                obsDistance = self.distance(self.tx[-1], self.ty[-1], self.ix[-1] + round(i * math.cos(self.theta[-1])), self.iy[-1] + round(i * math.sin(self.theta[-1])))
                break
        # print(f'obstacle Distance: {obsDistance}')
        return obsDistance

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
        path_distance = numpy.sqrt(x*x + y*y)
        win = 100 if self.distance(self.tx[-1], self.ty[-1], self.gx, self.gy) < self.dist_tolerance else 0
        return 0.1*self.getObstacle() - path_distance - self.getGoal() + self.getTheta() * self.robot_radius - 1 + win
    
    def isDone(self):
        term = False
        if self.distance(self.tx[-1], self.ty[-1], self.gx, self.gy) < self.dist_tolerance:
            term = True
        if self.distance(self.tx[-1], self.ty[-1], self.rx[self.pnt], self.ry[self.pnt]) > 5 * self.robot_radius:
            term = True
        if self.getObstacle() <= self.dist_tolerance:
            term = True
        return term

    def step(self, action):
        vLeft, vRight = action[0], action[1]
        dt = 0.1
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
        return [path_x, path_y, self.getGoal(), self.getObstacle(), self.getTheta(), theta], self.getReward(), self.isDone()

    def print(self):
        plt.plot(self.ox, self.oy, ".k")
        plt.plot(self.sx, self.sy, "og")
        plt.plot(self.gx, self.gy, "xb")
        plt.plot(self.rx, self.ry, '-r')
        plt.grid(True)
        plt.axis("equal")
        plt.plot(self.tx, self.ty, "-g")     
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
        return [path_x, path_y, self.getGoal(), self.getObstacle(), self.getTheta(), self.theta[-1]]

def main():
    print(__file__ + " start!!")
            # start and goal position
    sx = 10  # [m]
    sy = 10  # [m]
    gx = 50  # [m]
    gy = 50  # [m]
    grid_size = 71  # [m]
    robot_radius = 1.0  # [m]
    obstacle_count = 25

    sim = Simulation(robot_radius, grid_size, obstacle_count, sx, sy, gx, gy)

    agent = Agent(
    alpha=0.000025,
    beta=0.00025,
    input_dims=[6],
    tau=0.001,
    batch_size=64,
    fc1_dims=400,
    fc2_dims=300,
    n_actions=2,
    action_range=1
    )

    print(T.cuda.is_available())
    np.random.seed(0)

    score_history = []
    for i in range(1000):
        done = False
        score = 0
        obs = sim.reset()
        while not done:
            act = agent.choose_action(obs)
            new_state, reward, done = sim.step(act)
            print(new_state)
            agent.remember(obs, act, reward, new_state, int(done))
            agent.learn()
            score += reward
            obs = new_state


        score_history.append(score)
        print(
            "episode",
            i,
            "score %.2f" % score,
            "100 game average %.2f" % np.mean(score_history[-100:]),
        )

        if i % 25 == 0:
            agent.save_models()




if __name__ == '__main__':
    main()
