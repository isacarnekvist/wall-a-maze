import numpy as np
from numpy import inf
from math import cos, sin, pi
from scipy.spatial import distance


class Obstacle:
    pass


class PickableObstacle(Obstacle):
    pass


class Map:
    
    def __init__(self, map_path=None):
        self.edges = []

        # Read map from text file
        if map_path:
            with open(map_path) as f:
                for line in f:
                    if line[0] == '#':
                        continue
                    self.add_edge(*[float(c) for c in line.split()])
                    
    def add_obstacle(self, obstacle):
        pass
    
    def add_edge(self, x1, y1, x2, y2):
        self.edges.append((x1, y1, x2, y2))
        
    def scan_one(self, x, y, theta, alpha):
        """returns distances for one angle
        x, y: float
             position in meters
        theta: float
            robot rotation in radians
        alpha: float
            in what direction, relative to robot, to scan
        """
        n = len(self.edges)
        distances = np.zeros(n)
        for i, edge in enumerate(self.edges):
            dist = self._distance(x, y, theta, alpha, *edge)
            distances[i] = dist
        
        return np.min(distances)
    
    def _distance(self, x1, x2, theta, alpha, a1, a2, b1, b2):
        """
        Returns distance given one position, one edge, and an angle alpha
        """
        a = np.array([[a1, a2]]).T
        b = np.array([[b1, b2]]).T
        x = np.array([[x1, x2]]).T
        try:
            k = 1.0 / ((a2 - b2) * cos(theta + alpha) + sin(theta + alpha) * (b1 - a1))
            A = np.array([
                [ a2 - b2   , b1 - a1   ],
                [-sin(theta + alpha), cos(theta + alpha)]
            ])
            s, t = (k * np.dot(A, (a - x))).flatten()
            if 0 <= t <= 1 and 0 < s:
                return s
        except ZeroDivisionError:
            pass
        return np.inf
