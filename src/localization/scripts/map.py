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
                    
    def add_obstacle(obstacle):
        pass
    
    def add_edge(self, x1, y1, x2, y2):
        self.edges.append((x1, y1, x2, y2))
    
    def scan(self, x, y):
        """returns distances and derivatives in 360 degrees"""
        n = len(self.edges)
        distances = np.zeros((n, 360))
        dxs = np.zeros((n, 360))
        dys = np.zeros((n, 360))
        dthetas = np.zeros((n, 360))
        for i, edge in enumerate(self.edges):
            dist, dx, dy, dtheta = self._intersections(x, y, *edge)
            dxs[i, :] = dx
            dys[i, :] = dy
            dthetas[i, :] = dtheta
            distances[i, :] = dist
        
        min_inds = distances.argmin(axis=0)
        return (
            distances[min_inds, range(360)],
            dxs[min_inds, range(360)],
            dys[min_inds, range(360)],
            dthetas[min_inds, range(360)]
        )

    def _intersections(self, x1, x2, a1, a2, b1, b2):
        """
        Returns distances and derivatives given one position and one edge
        """
        distances = np.array([inf] * 360)
        dx1 = np.array([inf] * 360)
        dx2 = np.array([inf] * 360)
        dtheta = np.array([inf] * 360)
        a = np.array([[a1, a2]]).T
        b = np.array([[b1, b2]]).T
        x = np.array([[x1, x2]]).T
        for angle in range(360):
            theta = pi * angle / 180
            try:
                k = 1.0 / ((a2 - b2) * cos(theta) + sin(theta) * (b1 - a1))
                A = np.array([
                    [ a2 - b2   , b1 - a1   ],
                    [-sin(theta), cos(theta)]
                ])
                s, t = (k * np.dot(A, (a - x))).flatten()
                if 0 <= t <= 1 and 0 < s:
                    distances[angle] = s
                    dx1[angle] = k * (b2 - a2)
                    dx2[angle] = k * (a1 - b1)
                    f = (a1 - x1) * (a2 - b2) + (a2 - x2) * (b1 - a1)
                    fprim = np.array([cos(theta), sin(theta)]).dot(x - a)
                    g = 1 / k
                    gprim = np.array([cos(theta), sin(theta)]).dot(b - a)
                    dtheta[angle] = k ** 2 * (-gprim * f)
            except ZeroDivisionError:
                pass
        
        return distances, dx1, dx2, dtheta
