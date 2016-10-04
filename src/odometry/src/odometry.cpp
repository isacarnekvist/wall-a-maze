#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "phidgets/motor_encoder.h"
#include "std_msgs/Header.h"
#define PI           3.14159265358979323846  /* pi */

#include <algorithm>
#include <iostream>

double leftTicks = 0;
double rightTicks = 0;
double leftTimeSec[2] = {0.0, 0.0};
double rightTimeSec[2] = {0.0, 0.0};
double leftTimeNano[2] = {0.0, 0.0};
double rightTimeNano[2] = {0.0, 0.0};

double ticksPerRevolution = 900.0;
double r = 0.036;
double b = 0.215;

void leftEncoder(const phidgets::motor_encoder::ConstPtr& msg) {
    leftTimeNano[0] = leftTimeNano[1];
    leftTimeNano[1] = msg->header.stamp.nsec;

    leftTimeSec[0] = leftTimeSec[1];
    leftTimeSec[1] = msg->header.stamp.sec;

    leftTicks = msg->count_change;
}

void rightEncoder(const phidgets::motor_encoder::ConstPtr& msg) {
    rightTimeNano[0] = rightTimeNano[1];
    rightTimeNano[1] = msg->header.stamp.nsec;

    rightTimeSec[0] = rightTimeSec[1];
    rightTimeSec[1] = msg->header.stamp.sec;

    rightTicks = -msg->count_change;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "odometry");

    ros::NodeHandle n;

    ros::Subscriber left_encoder_sub = n.subscribe("/left_motor/encoder", 1000, leftEncoder);
    ros::Subscriber right_encoder_sub = n.subscribe("/right_motor/encoder", 1000, rightEncoder);

    ros::Publisher odometry_pub = n.advertise<geometry_msgs::Twist>("/odometry", 1000);

    ros::Rate loop_rate(125);

    geometry_msgs::Twist msg;
    while (ros::ok()) {
        if (leftTimeNano[0] == 0 || rightTimeNano[0] == 0) {
            ros::spinOnce();
            loop_rate.sleep();
            continue;
        }

        double leftDeltaTime = leftTimeSec[1] - leftTimeSec[0] + ((leftTimeNano[1] - leftTimeNano[0]) / 1e9);
        double rightDeltaTime = rightTimeSec[1] - rightTimeSec[0] + ((rightTimeNano[1] - rightTimeNano[0]) / 1e9);

        ROS_INFO("Time delta left: %f", leftDeltaTime);
        ROS_INFO("Time delta right: %f", rightDeltaTime);


        double wr = rightTicks * 2.0 * PI / (ticksPerRevolution*rightDeltaTime);
        double wl = leftTicks * 2.0 * PI / (ticksPerRevolution*leftDeltaTime);

        double vel = ((wr + wl) * r) / 2.0;
        double ang_vel = ((wr - wl) * r) / b;

        msg.linear.x = vel;
        msg.angular.z = ang_vel;

        odometry_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
