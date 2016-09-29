#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "phidgets/motor_encoder.h"
#include "std_msgs/Header.h"
#define PI           3.14159265358979323846  /* pi */

#include <algorithm>
#include <iostream>

double leftTicks = 0;
double rightTicks = 0;
double leftTime[2] = {0.0, 0.0};
double rightTime[2] = {0.0, 0.0};

double ticksPerRevolution = 83.0;
double r = 0.036;
double b = 0.215;

void leftEncoder(const phidgets::motor_encoder::ConstPtr& msg) {
    leftTime[0] = leftTime[1];
    leftTime[1] = msg->header.stamp.nsec;

    leftTicks = msg->count_change;
}

void rightEncoder(const phidgets::motor_encoder::ConstPtr& msg) {
    rightTime[0] = rightTime[1];
    rightTime[1] = msg->header.stamp.nsec;

    rightTicks = -msg->count_change;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "odometry");

    ros::NodeHandle n;

    ros::Subscriber left_encoder_sub = n.subscribe("/left_motor/encoder", 1000, leftEncoder);
    ros::Subscriber right_encoder_sub = n.subscribe("/right_motor/encoder", 1000, rightEncoder);

    ros::Publisher odometry_pub = n.advertise<geometry_msgs::Twist>("/odometry", 1000);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist msg;
    while (ros::ok()) {
        if (leftTime[0] == 0 || rightTime[0] == 0) {
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }

        double leftDeltaTime = (leftTime[1] - leftTime[0]) / 1e9;
        double rightDeltaTime = (rightTime[1] - rightTime[0]) / 1e9;

        double wr = 0.0;
        double wl = 0.0;

        wr = rightTicks*2*PI*r/(ticksPerRevolution*rightDeltaTime);
        wl = leftTicks*2*PI*r/(ticksPerRevolution*leftDeltaTime);

        double vel = 0.0;
        double ang_vel = 0.0;

        vel = ((wr+wl)*r)/2;
        ang_vel = ((wr-wl)*r)/b;

        msg.linear.x = vel;
        msg.angular.z = ang_vel;

        odometry_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
