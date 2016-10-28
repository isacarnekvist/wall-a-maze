#include <math.h>
#include <random>
#include <iostream>
#include <algorithm>

#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>

#include "map.hpp"
#include "stats.hpp"
#include "constants.hpp"
#include "particle_filter.hpp"

using namespace std;

default_random_engine random_engine;
normal_distribution<float> normal_sampler (0, 1);

void Particle::move(float linear, float angular, float delta_seconds) {
    /* Here are parameters that probably will need tuning! */
    float k = (abs(linear) * 0.01 + abs(angular) * 0.01 + 0.10);
    float theta_mean = theta; // + angular / 2;

    /* Update with noise! */
    x += (linear * cos(theta_mean) + k * normal_sampler(random_engine)) * delta_seconds;
    y += (linear * sin(theta_mean) + k * normal_sampler(random_engine)) * delta_seconds;
    theta += (angular + 0.1 * k * normal_sampler(random_engine)) * delta_seconds;
}

float Particle::likelihood(const Map &map, const vector<tuple<float, float> > &scans) {
    vector<float> discrepancies;
    float angle, scan, discrepancy_sum;
    discrepancy_sum = 0.0;
    for (const tuple<float, float> &t : scans) {
        angle = get<0>(t);
        scan = get<1>(t);
        if (abs(scan) == INF) {
            continue;
        }
        discrepancies.push_back(abs(scan - map.distance(x, y, theta + angle)));
    }
    sort(discrepancies.begin(), discrepancies.end());
    for (int i = 0; i < 40; i++) {
        discrepancy_sum += discrepancies[i];
    }
    return exp(-(discrepancy_sum));
}

/* For debugging purposes simulating a robot with measurements */
vector<tuple<float, float> > Particle::scan(const Map &map) {
    vector<tuple<float, float> > res;
    for (int i = 0; i < 360; i += 4) {
        float angle = M_PI * i / 180;
        float distance = map.distance(x, y, theta + angle) + 0.05 * normal_sampler(random_engine);
        if (normal_sampler(random_engine) > 1.2) {
            distance = INF;
        }
        res.push_back(
            make_tuple(angle, distance)
        );
    }
    return res;
}

void Particle::printParticle() {
    cout << "Particle at: " << x << ", " << y << ", " << theta << endl;
}

ParticleFilter::ParticleFilter(
    int n_particles, float min_x, float max_x, float min_y, float max_y, float min_theta, float max_theta
) : n_particles(n_particles), min_x(min_x), max_x(max_x), min_y(min_y), max_y(max_y), min_theta(min_theta), max_theta(max_theta)
{
    normal_sampler = normal_distribution<float>(0, 1);
    particles = vector<Particle>();
    sampleParticles(n_particles);
}

void ParticleFilter::move(float linear, float angular, float delta_seconds) {
    for (Particle &p : particles) {
        p.move(linear, angular, delta_seconds);
    }
}

void ParticleFilter::resample(const Map &map, const vector<tuple<float, float> > &scans) {
    vector<float> probabilities (n_particles);
    vector<Particle> old_particles = particles;
    for (int i = 0; i < n_particles; i++) {
        probabilities[i] = particles[i].likelihood(map, scans);
    }
    int n_noise_particles = 4;
    vector<int> resample_counts = multinomial_sample(n_particles - n_noise_particles, probabilities);
    particles = vector<Particle>();
    for (int i = 0; i < n_particles; i++) {
        for (int n_samples = 0; n_samples < resample_counts[i]; n_samples++) {
            particles.push_back(old_particles[i]);
        }
    }
    sampleParticles(n_noise_particles);
}

void ParticleFilter::sampleParticles(int n_particles) {
    uniform_real_distribution<float> x_sampler (min_x, max_x);
    uniform_real_distribution<float> y_sampler (min_y, max_y);
    uniform_real_distribution<float> theta_sampler (min_theta, max_theta);
    for (int i = 0; i < n_particles; i++) {
        particles.push_back(
            Particle(x_sampler(random_engine), y_sampler(random_engine), theta_sampler(random_engine))
        );
    }
}

float ParticleFilter::mean_estimate_x() {
    float x_sum = 0.0;
    for (Particle &p : particles) {
        x_sum += p.x;
    }
    return x_sum / n_particles;
}

float ParticleFilter::mean_estimate_y() {
    float y_sum = 0.0;
    for (Particle &p : particles) {
        y_sum += p.y;
    }
    return y_sum / n_particles;
}

float ParticleFilter::mean_estimate_theta() {
    float cos_sum = 0.0;
    float sin_sum = 0.0;
    for (Particle &p : particles) {
        cos_sum += cos(p.theta);
        sin_sum += sin(p.theta);
    }
    return atan2(sin_sum, cos_sum);
}

void ParticleFilter::printParticles() {
    for (Particle &p : particles) {
        p.printParticle();
    }
}
