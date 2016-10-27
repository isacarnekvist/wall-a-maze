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
    float k = (linear * 0.3 + angular * 0.03 + 0.01) * delta_seconds;
    float theta_mean = theta; // + angular / 2;

    /* Update with noise! */
    x += linear * cos(theta_mean) + k * normal_sampler(random_engine);
    y += linear * sin(theta_mean) + k * normal_sampler(random_engine);
    theta += angular + k * normal_sampler(random_engine); /* Use less/more noise for angle? */
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
    for (int i = 0; i < 20; i++) {
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
    sampleParticles();
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
    vector<int> resample_counts = multinomial_sample(n_particles, probabilities);
    particles = vector<Particle>();
    for (int i = 0; i < n_particles; i++) {
        for (int n_samples = 0; n_samples < resample_counts[i]; n_samples++) {
            particles.push_back(old_particles[i]);
        }
    }
}

void ParticleFilter::sampleParticles() {
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

int main(int argc, char **argv) {
    ros::init(argc, argv, "localization");
    ros::NodeHandle node_handle;
    ros::Publisher particle_publisher = node_handle.advertise<sensor_msgs::PointCloud>("/particles", 10);
    ros::Publisher position_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/position", 10);

    Map map = Map();
    ParticleFilter pf = ParticleFilter(2048, 0, 3, 0, 3, 0, 2 * M_PI);
    double x, y, theta;
    Particle robot (0.4, 0.15, 1.1);
    float distance = 0.05;
    float theta_delta = -0.02;
    for (int i = 0; i < 50; i++) {
        robot.x += cos(robot.theta) * distance;
        robot.y += sin(robot.theta) * distance;
        robot.theta += theta_delta;
        pf.move(distance, theta_delta, 1.5);
        cout << "Truth:" << endl;
        robot.printParticle();
        pf.resample(map, robot.scan(map));

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";
        pose.pose.position.x = robot.x;
        pose.pose.position.y = robot.y;
        pose.pose.orientation.x = cos(robot.theta / 2);
        pose.pose.orientation.y = sin(robot.theta / 2);

        sensor_msgs::PointCloud pcl;
        pcl.header.stamp = ros::Time::now();
        pcl.header.frame_id = "map";
        pcl.points.resize(pf.n_particles);
        for (int i = 0; i < pf.n_particles; i++) {
            Particle &p = pf.particles[i];
            geometry_msgs::Point32 point;
            point.x = p.x;
            point.y = p.y;
            point.z = 0.0;
            pcl.points[i] = point;
        }
        particle_publisher.publish(pcl);
        position_publisher.publish(pose);

        cout << "Estimate:" << endl;
        cout << pf.mean_estimate_x() << " " << pf.mean_estimate_y() << " " << pf.mean_estimate_theta() << endl;
        cout << endl;
    }
    return 0;
}
