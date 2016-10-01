#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "phidgets/motor_encoder.h"
#include "std_msgs/Float32.h"
# define PI           3.14159265358979323846  /* pi */

// Robot
double wheel_radius;
double wheel_base;

// PID-controller
// Left
double left_kp;
double left_ki;
double left_kd;
// Right
double right_kp;
double right_ki;
double right_kd;
// Other
double left_previous_error = 0;
double right_previous_error = 0;
double left_integral = 0;
double right_integral = 0;

double control_frequency;
double ticks_per_rev;

double left_desired;
double right_desired;

double left_estimated;
double right_estimated;

ros::Publisher left_pub;
ros::Publisher right_pub;
ros::Subscriber twist_sub;
ros::Subscriber left_encoder_sub;
ros::Subscriber right_encoder_sub;

// How we wish to move
void twistCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    double linear_velocity = msg->linear.x;
    double angular_velocity = msg->angular.z;

    left_desired = (linear_velocity - ((wheel_base/2.0) * angular_velocity)) / wheel_radius;
    right_desired = -(linear_velocity + ((wheel_base/2.0) * angular_velocity)) / wheel_radius;
}

// Encoder feedback signals
void leftEncoderCallback(const phidgets::motor_encoder::ConstPtr& msg) {
    double delta_encoder = (double) msg->count_change;

    // TODO: Maybe we need to calculate the delta time?!

    left_estimated = (delta_encoder * 2.0 * PI * control_frequency) / ticks_per_rev;
}

void rightEncoderCallback(const phidgets::motor_encoder::ConstPtr& msg) {
    double delta_encoder = (double) msg->count_change;

    // TODO: Maybe we need to calculate the delta time?!

    right_estimated = (delta_encoder * 2.0 * PI * control_frequency) / ticks_per_rev;
}

void PIDController(int * left_pwm, int * right_pwm) {
    double left_error = left_desired - left_estimated;
    double right_error = right_desired - right_estimated;

    double dt = 1.0/control_frequency;

    left_integral += left_error * dt;
    right_integral += right_error * dt;

    double left_derivative = (left_error - left_previous_error)/dt;
    double right_derivative = (right_error - right_previous_error)/dt;

    *left_pwm = left_kp * left_error + left_ki * left_integral + left_kd * left_derivative;
    *right_pwm = right_kp * right_error + right_ki * right_integral + right_kd * right_derivative;

    left_previous_error = left_error;
    right_previous_error = right_error;
}

void initParams(ros::NodeHandle n) {
    // Robot
    n.param<double>("/robot/wheel_radius", wheel_radius, 0.0352);
    n.param<double>("/robot/wheel_base", wheel_base, 0.23);

    // PID-controller
    // Left
    n.param<double>("/left_pid_controller/kp", left_kp, 4.4);
    n.param<double>("/left_pid_controller/ki", left_ki, 0.3);
    n.param<double>("/left_pid_controller/kd", left_kd, 0.5);
    // Right
    n.param<double>("/right_pid_controller/kp", right_kp, 4);
    n.param<double>("/right_pid_controller/ki", right_ki, 0.3);
    n.param<double>("/right_pid_controller/kd", right_kd, 0.5);

    ROS_INFO("left_kp: %f, left_ki: %f, left_kd: %f, right_kp: %f, right_ki: %f, right_kd: %f", left_kp, left_ki, left_kd, right_kp, right_ki, right_kd);

    // Other
    n.param<double>("/control_frequency", control_frequency, 10);
    n.param<double>("/ticks_per_rev", ticks_per_rev, 360);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "wheels_controller");

    ros::NodeHandle n;

    initParams(n);

    left_pub = n.advertise<std_msgs::Float32>("/left_motor/cmd_vel", 1000);
    right_pub = n.advertise<std_msgs::Float32>("/right_motor/cmd_vel", 1000);

    twist_sub = n.subscribe("/motor_controller", 1000, twistCallback);
    left_encoder_sub = n.subscribe("/left_motor/encoder", 1000, leftEncoderCallback);
    right_encoder_sub = n.subscribe("/right_motor/encoder", 1000, rightEncoderCallback);


    ros::Rate loop_rate(control_frequency);

    std_msgs::Float32 left_msg;
    std_msgs::Float32 right_msg;

    int left_pwm, right_pwm;

    while (ros::ok()) {
        if (left_desired * left_desired < 0.001 && right_desired * right_desired < 0.001) {
            left_msg.data = 0;
            right_msg.data = 0;
            left_pub.publish(left_msg);
            right_pub.publish(right_msg);

            ros::spinOnce();

            loop_rate.sleep();

            continue;
        }

        PIDController(&left_pwm, &right_pwm);

        left_msg.data += left_pwm;
        right_msg.data += right_pwm;

        ROS_INFO("\nLeft:\tDesired %f\tEstimated %f\tPower %f\nRight:\tDesired %f\tEstimated %f\tPower %f", left_desired, left_estimated, left_msg.data, right_desired, right_estimated, right_msg.data);

        if (left_msg.data > 50) {
            double difference = right_msg.data / left_msg.data;
            left_msg.data = 50.0;
            right_msg.data = left_msg.data * difference;
        } else if (right_msg.data > 50) {
            double difference = left_msg.data / right_msg.data;
            right_msg.data = 50.0;
            left_msg.data = right_msg.data * difference;
        } else if (left_msg.data < -50) {
            double difference = right_msg.data/left_msg.data;
            left_msg.data = -50.0;
            right_msg.data = left_msg.data * difference;
        } else if (right_msg.data < -50) {
            double difference = left_msg.data/right_msg.data;
            right_msg.data = -50.0;
            left_msg.data = right_msg.data * difference;
        }


        //left_msg.data = 15;
        //right_msg.data = -15;

        left_pub.publish(left_msg);
        right_pub.publish(right_msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
