from __future__ import print_function
import Queue
from copy import deepcopy

import numpy as np
import networkx


class OccupancyGrid:
    
    def __init__(self, x_min, x_max, y_min, y_max, cell_width, padding):
        """
        Parameters
        ==========
        cell_width : float
            the resolution of the grid map, in meters
        padding : float
            the extra space to add outside the map, in meters
        """
        self.x_min = x_min
        self.y_min = y_min
        self.x_max = x_max
        self.y_max = y_max
        self.padding = padding
        self.cell_width = cell_width
        self.n_width = int((x_max - x_min) / cell_width + 1)
        self.n_height = int((y_max - y_min) / cell_width + 1)
        self._grid = np.zeros((self.n_height, self.n_width))
        
    def occupy_line(self, x1, y1, x2, y2):
        a = np.array([x1, y1])
        v = np.array([x2 - x1, y2 - y1])
        for t in np.linspace(0, 1, self.n_width + self.n_height):
            p = a + t * v
            self.occupy(p[0], p[1])
            
    def expand_obstacles(self, w):
        if w > self.padding:
            raise ValueError('Padding needs to be > expand size atm')
        grid_copy = self._grid + 0
        for degree in np.linspace(0, 2 * np.pi, 180):
            x_displace = int(np.cos(degree) * w / self.cell_width)
            y_displace = int(np.sin(degree) * w / self.cell_width)
            x_shifted = np.roll(grid_copy, x_displace, axis=0)
            shifted = np.roll(x_shifted, y_displace, axis=1)
            self._grid += shifted
                    
        self._grid[self._grid > 0] = 1
        
    def occupied(self, x_ind, y_ind):
        return self._grid[y_ind, x_ind] == 1.0
    
    def closest_non_occupied(self, x, y):
        for t in np.linspace(0, 1.5, 20):
            for angle in np.linspace(0, 2 * np.pi):
                xp = x + t * np.cos(angle)
                yp = y + t * np.sin(angle)
                x_ind, y_ind = self.coord_to_inds(xp, yp)
                if not self.occupied(x_ind, y_ind):
                    return xp, yp
            
    def occupy(self, x, y):
        x_ind, y_ind = self.coord_to_inds(x, y)
        self._grid[y_ind, x_ind] = 1.0
        
    def line_obstructed(self, x1, y1, x2, y2):
        """
        arguments are indices
        """
        for t in np.linspace(0, 1, self.n_width + self.n_height):
            if self.occupied(int(x1 + t * (x2 - x1)), int(y1 + t * (y2 - y1))):
                return True
        return False
    
    def coord_to_inds(self, x, y):
        x_ind = int((x + self.padding + self.cell_width / 2) / self.cell_width)
        y_ind = int((y + self.padding + self.cell_width / 2) / self.cell_width)
        return x_ind, y_ind
    
    def inds_to_coords(self, x, y):
        return (
            x * self.cell_width - self.padding,
            y * self.cell_width - self.padding
        )
    
    @property
    def grid(self):
        return deepcopy(self._grid)
        
    def to_graph(self):
        g = networkx.Graph()
        # iterate over indices
        for x in range(1, self.n_width - 1):
            for y in range(1, self.n_height - 1):
                if self.occupied(x, y):
                    continue
                for vert in [-1, 0, 1]:
                    for horiz in [-1, 0, 1]:
                        if abs(vert) == abs(horiz):
                            continue
                        if self.occupied(x + horiz, y + vert):
                            continue
                        g.add_edge((x, y), (x + horiz, y + vert))
        return g
        
        
def lines_to_grid(lines, cell_width=0.03, padding=0.4):
    x_max = -np.inf
    x_min = np.inf
    y_max = -np.inf
    y_min = np.inf
    for x1, y1, x2, y2 in lines:
        x_max = max(x1 + padding, x2 + padding, x_max)
        x_min = min(x1 - padding, x2 - padding, x_min)
        y_max = max(y1 + padding, y2 + padding, y_max)
        y_min = min(y1 - padding, y2 - padding, y_min)
    o = OccupancyGrid(x_min, x_max, y_min, y_max, cell_width, padding)
    for x1, y1, x2, y2 in lines:
        o.occupy_line(x1, y1, x2, y2)
    return o


def shortest_path_bfs(g, start, goal):
    q = Queue.Queue()
    preds = {start: None}
    q.put(start)
    while not q.empty():
        c = q.get()
        if c == goal:
            break
        for n in g[c]:
            if n not in preds:
                q.put(n)
                preds[n] = c
    if c != goal:
        raise ValueError('No path found')
    path = [goal]
    c = goal
    while c != start:
        c = preds[c]
        path.append(c)
    return list(reversed(path))


def euler_path_plan(x1, y1, x2, y2, grid, graph):
    x1, y1 = grid.closest_non_occupied(x1, y1)
    x1_ind, y1_ind = grid.coord_to_inds(x1, y1)
    x2, y2 = grid.closest_non_occupied(x2, y2)
    x2_ind, y2_ind = grid.coord_to_inds(x2, y2)
    shortest_path = shortest_path_bfs(
        graph,
        (x1_ind, y1_ind),
        (x2_ind, y2_ind)
    )
    a, b = 0, 1
    smoothed = []
    while b < len(shortest_path):
        x0, y0 = shortest_path[a]
        x1, y1 = shortest_path[b]
        if grid.line_obstructed(x0, y0, x1, y1):
            smoothed.append(b-1)
            a = b - 1
            continue
        b += 1
    smoothed.append(len(shortest_path) - 1)
    
    return [
        grid.inds_to_coords(shortest_path[ind][0], shortest_path[ind][1])
        for ind in smoothed
    ]
