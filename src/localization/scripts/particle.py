import numpy as np


class Particle:
    
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        
    def update(self, forward_delta, delta_theta):
        self.theta += delta_theta + np.random.normal(0, 0.08)
        forward_noise = np.random.normal(0, abs(forward_delta * 0.1) + 0.1)
        self.x += np.cos(self.theta) * (forward_delta + forward_noise)
        self.y += np.sin(self.theta) * (forward_delta + forward_noise)
        
    def __repr__(self):
        return 'Particle at: {} {} {}'.format(self.x, self.y, self.theta)
    
    def likelihood(self, lidar_coords, map_):
        dists = []
        for x, y in lidar_coords:
            if np.abs(x) == np.inf or np.abs(y) == np.inf:
                continue
            alpha = atan2(-x, y)
            dist = map_.scan_one(self.x, self.y, self.theta, alpha)
            #                 measured             expected
            dists.append(abs(np.sqrt(x ** 2 + y ** 2) - dist))
        dists.sort()
        return np.exp(-sum(dists[:20]))


class ParticleFilter:
    
    def __init__(
        self,
        n_particles=32,
        xlim=(0, 2.4),
        ylim=(0, 2.5),
        thetalim=(0, 2 * np.pi)
    ):
        self.particles = [
            Particle(
                np.random.rand() * (xlim[1] - xlim[0]) + xlim[0],
                np.random.rand() * (ylim[1] - ylim[0]) + ylim[0],
                np.random.rand() * (thetalim[1] - thetalim[0]) + thetalim[0],
            )
            for _ in range(n_particles)
        ]
            
    def resample(self, lidar_coords, map_, n_samples):
        weights = np.zeros(len(self.particles))
        for i, p in enumerate(self.particles):
            weights[i] = p.likelihood(lidar_coords, map_)
        weights /= weights.sum()
        samples = np.random.multinomial(n_samples, weights)
        
        new_particles = []
        for particle_index, n_copies in enumerate(samples):
            for n in range(n_copies):
                new_particles.append(deepcopy(self.particles[particle_index]))
                
        self.particles = new_particles
            
    def update(self, delta_forward, delta_theta):
        for p in self.particles:
            p.update(delta_forward, delta_theta)
