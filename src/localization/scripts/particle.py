class Particle:
    
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        
    def update(self, delta_forward, delta_theta):
        self.theta += delta_theta + np.random.normal(0, 0.08)
        forward_noise = np.random.normal(0, 0.03)
        self.x += np.cos(self.theta) * delta_forward + forward_noise
        self.y += np.sin(self.theta) * delta_forward + forward_noise
        
    def __repr__(self):
        return 'Particle at: {} {} {}'.format(self.x, self.y, self.theta)
