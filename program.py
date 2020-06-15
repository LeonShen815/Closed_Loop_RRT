import matplotlib.pyplot as plt
from random import uniform
from math import sqrt, atan2, cos, sin


def main():
    bounds = [
        (0, 0),
        (25, 0),
        (25, 25),
        (0, 25)
    ]

    ws = Workspace(bounds)
    ws.plot()

    x0 = State(5, 5)
    xf = State(20, 20)

    robot = Robot(x0, xf)

    for i in range(0, 100):
        tree_expansion(ws, robot, 1)

    robot.plot_tree()
    robot.plot()
    robot.plot_goal()
    plt.axis('equal')
    plt.show()


def tree_expansion(ws, robot, step=1):

    s_s = ws.return_sample_space()

    x_s = uniform(s_s[0][0], s_s[0][1])
    y_s = uniform(s_s[1][0], s_s[1][1])

    x_s = State(x_s, y_s)

    # ID nearest node
    n_near = robot.tree[0][0]
    min_dist = n_near.return_distance(x_s)

    for node, _ in robot.tree:
        if node.return_distance(x_s) < min_dist:
            n_near = node
            min_dist = n_near.return_distance(x_s)

    # expand towards nearest node
    if min_dist < step:
        n_new = x_s
    else:
        angle = n_near.return_angle(x_s)
        x_new = n_near.x + step*cos(angle)
        y_new = n_near.y + step*sin(angle)
        n_new = State(x_new, y_new)

    robot.tree.append((n_new, n_near))


class State:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def plot(self, color='k'):
        plt.plot(self.x, self.y, 'x', mec=color)

    def return_distance(self, node):
        return sqrt((self.x - node.x)**2 + (self.y - node.y)**2)

    def return_angle(self, node):
        return atan2(node.y - self.y, node.x - self.x)


class Workspace:
    def __init__(self, bounds, obstacles=None):
        self.bounds = bounds
        self.obstacles = obstacles

    def plot(self):
        x_ords = []
        y_ords = []
        for coordinate in self.bounds:
            x_ords.append(coordinate[0])
            y_ords.append(coordinate[1])

        x_ords.append(x_ords[0])
        y_ords.append(y_ords[0])

        plt.plot(x_ords, y_ords)

    def return_sample_space(self):
        x_min = self.return_min_bound(0)
        x_max = self.return_max_bound(0)
        y_min = self.return_min_bound(1)
        y_max = self.return_max_bound(1)

        return [
            (x_min, x_max),
            (y_min, y_max)
        ]

    def return_min_bound(self, index):
        min_val = self.bounds[0][index]

        for coordinate in self.bounds:
            if coordinate[index] < min_val:
                min_val = coordinate[index]

        return min_val

    def return_max_bound(self, index):
        max_val = self.bounds[0][index]

        for coordinate in self.bounds:
            if coordinate[index] > max_val:
                max_val = coordinate[index]

        return max_val


class Robot:
    def __init__(self, x0, xf, color="darkred"):
        self.x0 = x0
        self.x = x0
        self.xf = xf
        self.tree = [(x0, x0)]
        self.color = color

    def plot(self):
        self.x.plot(self.color)

    def plot_goal(self, color="forestgreen"):
        self.xf.plot(color)

    def plot_tree(self, color='lightblue'):
        for node in self.tree:
            x_ords = [node[1].x, node[0].x]
            y_ords = [node[1].y, node[0].y]
            plt.plot(x_ords, y_ords, ls='-', c=color, marker='o', mfc=color, mec=color)


if __name__ == "__main__":
    main()
