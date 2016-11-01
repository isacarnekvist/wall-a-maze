#ifndef _PART_FILT_HPP
#define _PART_FILT_HPP

#include <tuple>
#include <vector>

using namespace std;

class Particle {
public:
    Particle(float x, float y, float theta) : x(x), y(y), theta(theta) {};
    void move(float linear, float angular, float delta_seconds);
    float x, y, theta;
    float likelihood(const Map &map, const vector<tuple<float, float> > &scans);
    void printParticle();
private:
    
};

class ParticleFilter {
public:
    ParticleFilter(int n_particles, float min_x, float max_x, float min_y, float max_y, float min_theta, float max_theta);
    void resample(const Map &map, const vector<tuple<float, float> > &scans);
    void move(float linear, float angular, float delta_seconds);
    void printParticles();
    float mean_estimate_x();
    float mean_estimate_y();
    float mean_estimate_theta();
    vector<Particle> particles; /* Make private? */
    int n_particles;
    float min_x, max_x, min_y, max_y, min_theta, max_theta;
private:
    void sampleParticles(int n_particles);
};

#endif
