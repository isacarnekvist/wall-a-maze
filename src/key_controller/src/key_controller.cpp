#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "keyop/Keyop.h"

#include <algorithm>

double speed = 0.2;
double speedIncrement = 0.1;

double maxSpeed = 1.0;
double minSpeed = 0.0;

bool forward = false;
bool right = false;
bool back = false;
bool left = false;

bool accelerate = false;
bool deaccelerate = false;


void key(const keyop::Keyop::ConstPtr& msg) {
	forward = msg->forward;
	right = msg->right;
	back = msg->back;
	left = msg->left;
	
	accelerate = msg->accelerate;
	deaccelerate = msg->deaccelerate;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "key_controller");
 
    ros::NodeHandle n;
 
    ros::Subscriber keyop_sub = n.subscribe("/keyop/Keyop", 1000, key);
 
    ros::Publisher keyop_pub = n.advertise<geometry_msgs::Twist>("/motor_controller", 1000);
 
    ros::Rate loop_rate(10);
 
	geometry_msgs::Twist msg;
    while (ros::ok()) {
    	if (accelerate) {
    		speed = std::min(maxSpeed, speed + speedIncrement);
    	}
    	if (deaccelerate) {
    		speed = std::max(minSpeed, speed - speedIncrement);
    	}
    	
    	if (forward && back) {
    		msg.linear.x = 0;
    	} else if (forward) {
    		msg.linear.x = speed;
    	} else if (back) {
    		msg.linear.x = -speed;
    	} else {
    		msg.linear.x = 0;
    	}
    	
    	if (right && left) {
    		msg.angular.z = 0;
    	} else if (right) {
    		msg.angular.z = -12 * speed;
    	} else if (left) {
    		msg.angular.z = 12 * speed;
    	} else {
    		msg.angular.z = 0;
    	}
 
        keyop_pub.publish(msg);
 
        ros::spinOnce();
 
        loop_rate.sleep();
    }
 
    return 0;
}
