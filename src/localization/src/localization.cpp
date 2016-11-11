#include <math.h>
#include <random>
#include <iostream>
#include <algorithm>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PoseStamped.h>
#include <planner/PlannerStatus.h>

#include "map.hpp"
#include "stats.hpp"
#include "constants.hpp"
#include "particle_filter.hpp"

using namespace std;

class Localization {
public:
    Localization(ros::NodeHandle &node_handle);
    ~Localization();
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void odometry_callback(const geometry_msgs::Twist::ConstPtr &msg);
    void publish_pose_estimate();
    void publish_updated_obstacles();
private:
    vector<tuple<float, float> > scans;
    ros::Publisher position_publisher;
    ros::Publisher particle_publisher;
    ros::Publisher obstacle_publisher;
    Map map;
    ParticleFilter *particle_filter; /* Doesn't work without '*' and new */
    bool recieved_laser;
    bool recieved_odometry;
    ros::Time odometry_stamp;
};

Localization::Localization(ros::NodeHandle &node_handle) {
    recieved_laser = false;
    recieved_odometry = false;
    position_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/position", 10);
    particle_publisher = node_handle.advertise<sensor_msgs::PointCloud>("/particles", 10);
    obstacle_publisher = node_handle.advertise<geometry_msgs::Polygon>("/obstacles", 1000);
    map = Map();
    particle_filter = new ParticleFilter(
        /* TODO Change these to be set from the map */
        512,       /* Number of particles */
        2.0,          /* x_min */
        2.2,       /* x_max */
        0.0,          /* y_min */
        0.4,       /* y_max */
        0,          /* theta_min */
        2 * M_PI    /* theta_max */
    );
}

Localization::~Localization() {
    delete particle_filter;
}

void Localization::publish_pose_estimate() {
    /* Reestimate */
    if (!recieved_laser) return;
    vector<tuple<float, float> > some_scans_n_shit = vector<tuple<float, float> >();
    for (int i = 0; i < scans.size(); i += 4) {
        some_scans_n_shit.push_back(scans[i]);
    }
    particle_filter->resample(map, some_scans_n_shit);

    /* Publish */
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";
    pose.pose.position.x = particle_filter->mean_estimate_x();
    pose.pose.position.y = particle_filter->mean_estimate_y();
    float theta_estimate = particle_filter->mean_estimate_theta();

    tf::Quaternion q;
    q.setRPY(0, 0, theta_estimate);
    tf::quaternionTFToMsg(q, pose.pose.orientation);

    position_publisher.publish(pose);

    sensor_msgs::PointCloud pcl;
    pcl.header.stamp = ros::Time::now();
    pcl.header.frame_id = "map";
    pcl.points.resize(particle_filter->n_particles);
    for (int i = 0; i < particle_filter->n_particles; i++) {
        Particle &p = particle_filter->particles[i];
        geometry_msgs::Point32 point;
        point.x = p.x;
        point.y = p.y;
        point.z = 0.0;
        pcl.points[i] = point;
    }
    particle_publisher.publish(pcl);
}

void Localization::publish_updated_obstacles() {
    geometry_msgs::Polygon res;
    geometry_msgs::Point32 p1;
    for (Wall w : map.walls) {
        geometry_msgs::Point32 p1;
        p1.x = w.x1;
        p1.y = w.y1;
        p1.z = 0;
        geometry_msgs::Point32 p2;
        p2.x = w.x2;
        p2.y = w.y2;
        p2.z = 0;
        res.points.push_back(p1);
        res.points.push_back(p2);
    }
    obstacle_publisher.publish(res);
}

void Localization::odometry_callback(const geometry_msgs::Twist::ConstPtr &msg) {
    float time_delta = 0.0;
    if (recieved_odometry) {
        time_delta = (ros::Time::now() - odometry_stamp).toSec();
    } else {
        recieved_odometry = true;
    }
    particle_filter->move(msg->linear.x, msg->angular.z, time_delta);
    odometry_stamp = ros::Time::now();
}

void Localization::laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    scans = vector<tuple<float, float> >();
    for (int degree = 0; degree < 360; degree++) {
        float distance = msg->ranges[degree];
        if (distance == INF || distance < 0.4) continue;
        float alpha = M_PI * (degree + 88.5) / 180.0;
        float x = distance * cos(alpha) + 0.08;
        float y = distance * sin(alpha) + 0.009;
        float alpha_prim = atan2(y, x);
        float dist_prim = sqrt(pow(x, 2) + pow(y, 2));
        scans.push_back(make_tuple(alpha_prim, dist_prim));
    }
    recieved_laser = true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "localization");
    ros::NodeHandle node_handle;
    Localization localization (node_handle);
    ros::Subscriber laser_subscriber = node_handle.subscribe(
        "/scan",
        10,
        &Localization::laser_callback,
        &localization
    );
    ros::Subscriber odometry_subscriber = node_handle.subscribe(
        "/odometry",
        10,
        &Localization::odometry_callback,
        &localization
    );

    ros::ServiceClient planner_status_client = node_handle.serviceClient<planner::PlannerStatus>("planner_ready");
    planner_status_client.waitForExistence();

    bool temp_updated_map = false;
    ros::Rate rate (10);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
        localization.publish_pose_estimate();
        if (!temp_updated_map) {
            localization.publish_updated_obstacles(); /* Move this so that it publishes when updated! */
            temp_updated_map = true;
        }
    }

    return 0;
}
