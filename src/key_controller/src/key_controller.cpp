#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "keyop/Keyop.h"

#include <algorithm>

double linear_velocity;
double linear_increment;

double linear_max;
double linear_min;

double angular_velocity;
double angular_increment;

double angular_max;
double angular_min;

bool forward = false;
bool right = false;
bool back = false;
bool left = false;

bool linear_accelerate = false;
bool linear_deaccelerate = false;
bool angular_accelerate = false;
bool angular_deaccelerate = false;

double control_frequency;

void initParams(ros::NodeHandle n) {
    n.getParam("/linear_velocity", linear_velocity);
    n.getParam("/linear_increment", linear_increment);
    n.getParam("/linear_max", linear_max);
    n.getParam("/linear_min", linear_min);
    n.getParam("/angular_velocity", angular_velocity);
    n.getParam("/angular_increment", angular_increment);
    n.getParam("/angular_max", angular_max);
    n.getParam("/angular_min", angular_min);

    n.getParam("/general/control_frequency", control_frequency);
}


void key(const keyop::Keyop::ConstPtr& msg) {
	forward = msg->forward;
	right = msg->right;
	back = msg->back;
	left = msg->left;
	
    linear_accelerate = msg->linear_accelerate;
    linear_deaccelerate = msg->linear_deaccelerate;
    angular_accelerate = msg->angular_accelerate;
    angular_deaccelerate = msg->angular_deaccelerate;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "key_controller");
 
    ros::NodeHandle n;

    initParams(n);
 
    ros::Subscriber keyop_sub = n.subscribe("/keyop/Keyop", 1000, key);
 
    ros::Publisher keyop_pub = n.advertise<geometry_msgs::Twist>("/motor_controller", 1000);
 
    ros::Rate loop_rate(control_frequency);
 
	geometry_msgs::Twist msg;
    while (ros::ok()) {
        /* Linear */
        if (linear_accelerate) {
            linear_velocity = std::min(linear_max, linear_velocity + linear_increment);
    	}
        if (linear_deaccelerate) {
            linear_velocity = std::max(linear_min, linear_velocity - linear_increment);
    	}
    	
    	if (forward && back) {
    		msg.linear.x = 0;
    	} else if (forward) {
            msg.linear.x = linear_velocity;
    	} else if (back) {
            msg.linear.x = -linear_velocity;
    	} else {
    		msg.linear.x = 0;
    	}

        /* Angular */
        if (angular_accelerate) {
            angular_velocity = std::min(angular_max, angular_velocity + angular_increment);
        }
        if (angular_deaccelerate) {
            angular_velocity = std::max(angular_min, angular_velocity - angular_increment);
        }
    	
    	if (right && left) {
    		msg.angular.z = 0;
    	} else if (right) {
            msg.angular.z = -angular_velocity;
    	} else if (left) {
            msg.angular.z = angular_velocity;
    	} else {
    		msg.angular.z = 0;
    	}
 
        keyop_pub.publish(msg);
 
        ros::spinOnce();
 
        loop_rate.sleep();
    }
 
    return 0;
}
