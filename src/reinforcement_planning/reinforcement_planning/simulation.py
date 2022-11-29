import math
from platform import node
from queue import PriorityQueue
import numpy as np
import time

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

        for i in range(70):
            self.nodes[i][0] = self.Node(i, 0, 0, 0, (0, 0), True)
        for i in range(70):
            self.nodes[70][i] = self.Node(70, i, 0, 0, (0, 0), True)
        for i in range(71):
            self.nodes[i][70] = self.Node(i, 70, 0, 0, (0, 0), True)
        for i in range(71):
            self.nodes[0][i] = self.Node(0, i, 0, 0, (0, 0), True)
        for i in range(50):
            self.nodes[30][i] = self.Node(30, i, 0, 0, (0, 0), True)
        for i in range(0, 40):
            self.nodes[50][70 - i] = self.Node(50, 70-i, 0, 0, (0, 0), True)

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
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

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
    def __init__(self, obstacles):
        self.obstacles = obstacles
    
    def step():
        minVal = distance(tx[-1], ty[-1], rx[0], ry[0])
        pnt = 0
        for i in range(1, len(rx)):
            new = distance(tx[-1], ty[-1], rx[i], ry[i])
            if new < minVal:
                pnt = i
                minVal = new
        pathDistance = (rx[pnt] - tx[-1], ry[pnt] - ty[-1])
        print(f'path distance: {pathDistance}')
        
        #distance to goal along path
        goalDistance = (gx - rx[pnt], gy - ry[pnt])
        print(f'goal: {goalDistance}')

        #distance to obstacle 
        obsDistance = (math.inf, math.inf)
        for i in range(10):
            if map[ix[-1] + round(i * math.cos(theta[-1])), iy[-1] + round(i * math.sin(theta[-1]))]:
                obsDistance = (ix[-1] + round(i * math.cos(theta[-1])) - tx[-1], iy[-1] + round(i * math.sin(theta[-1])) - ty[-1])
                break
        print(obsDistance)

        #angle difference
        pathVector = (rx[pnt + 1] - rx[pnt], ry[pnt + 1] - ry[pnt])
        robotVector = (math.cos(theta[-1]), math.sin(theta[-1]))

        stuff = (pathVector[0] * robotVector[0] + pathVector[1] * robotVector[1]) / (distance(pathVector[0], pathVector[1], 0, 0) * distance(robotVector[0], robotVector[1], 0,0))
        deltTheta = math.acos(min(stuff, 1))
        print(f'Delta Theta: {deltTheta}\n')
        theta.append(theta[-1] + deltTheta)

        ##replaced by finding the actual new point and going there
        tx.append(rx[pnt2])
        ty.append(ry[pnt2])
        ix.append(rx[pnt2])
        iy.append(ry[pnt2])
        pnt2 += 1

def distance(x1, y1, x2, y2):
    return math.sqrt((x2-x1)**2 + (y2-y1)**2)

def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = 10  # [m]
    sy = 10  # [m]
    gx = 50  # [m]
    gy = 50  # [m]
    grid_size = 71  # [m]
    robot_radius = 1.0  # [m]

    map = np.zeros((71, 71), dtype=bool)

    # set obstacle positions
    ox, oy = [], []
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
    for i in range(-10, 40):
        ox.append(20)
        oy.append(i)
        map[20, i + 10] = True
    for i in range(0, 40):
        ox.append(40)
        oy.append(60 - i)
        map[40, 70 - i] = True

    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
    rx, ry = a_star.planning(sx, sy, gx, gy, ox, oy)

    tx = [sx]
    ty = [sy]
    ix = [sx]
    iy = [sy]
    pnt2 = 1
    theta = [math.pi / 2]

    rx.reverse()
    ry.reverse()
    while(round(tx[-1]) != gx and round(ty[-1]) != gy):
        # get min distance from center robot and closest point of path 
        minVal = distance(tx[-1], ty[-1], rx[0], ry[0])
        pnt = 0
        for i in range(1, len(rx)):
            new = distance(tx[-1], ty[-1], rx[i], ry[i])
            if new < minVal:
                pnt = i
                minVal = new
        pathDistance = (rx[pnt] - tx[-1], ry[pnt] - ty[-1])
        print(f'path distance: {pathDistance}')
        
        #distance to goal along path
        goalDistance = (gx - rx[pnt], gy - ry[pnt])
        print(f'goal: {goalDistance}')

        #distance to obstacle 
        obsDistance = (math.inf, math.inf)
        for i in range(10):
            if map[ix[-1] + round(i * math.cos(theta[-1])), iy[-1] + round(i * math.sin(theta[-1]))]:
                obsDistance = (ix[-1] + round(i * math.cos(theta[-1])) - tx[-1], iy[-1] + round(i * math.sin(theta[-1])) - ty[-1])
                break
        print(obsDistance)

        #angle difference
        pathVector = (rx[pnt + 1] - rx[pnt], ry[pnt + 1] - ry[pnt])
        robotVector = (math.cos(theta[-1]), math.sin(theta[-1]))

        stuff = (pathVector[0] * robotVector[0] + pathVector[1] * robotVector[1]) / (distance(pathVector[0], pathVector[1], 0, 0) * distance(robotVector[0], robotVector[1], 0,0))
        deltTheta = math.acos(min(stuff, 1))
        print(f'Delta Theta: {deltTheta}\n')
        theta.append(theta[-1] + deltTheta)

        ##replaced by finding the actual new point and going there
        tx.append(rx[pnt2])
        ty.append(ry[pnt2])
        ix.append(rx[pnt2])
        iy.append(ry[pnt2])
        pnt2 += 1


    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.pause(0.001)
        plt.show()


if __name__ == '__main__':
    main()
