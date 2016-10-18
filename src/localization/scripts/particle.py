import numpy as np


class Particle:
    
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        
    def update(self, delta_x, delta_y, delta_theta):
        self.theta += delta_theta + np.random.normal(0, 0.08)
        self.x += delta_x + np.random.normal(0, abs(delta_x * 0.1 + 0.1))
        self.y += delta_y + np.random.normal(0, abs(delta_y * 0.1 + 0.1))
        
    def __repr__(self):
        return 'Particle at: {} {} {}'.format(self.x, self.y, self.theta)
