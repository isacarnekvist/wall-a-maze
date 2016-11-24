#include <math.h>
#include <iostream>

#include <tf/tf.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/server/simple_action_server.h>

#include <planner/PathPlan.h>
#include <planner/PlannerTargetAction.h>

const static float INF = std::numeric_limits<float>::infinity();

using namespace std;

/* LinePlan states */
enum LINEPLAN_STATE {
    INITIALIZED,
    INITIAL_ROTATION,
    BEFORE_LINE_EXECUTION,
    EXECUTING_LINE,
    BEFORE_TARGET_ROTATION,
    TARGET_ROTATION,
    BEFORE_DONE,
    DONE,
    OBSTRUCTED
};

typedef struct LinePlan {
    int state;
    float start_x;
    float start_y;
    float target_x;
    float target_y;
    float target_theta;
    bool ignore_target_theta;
    ros::Time timestamp; /* when was this last executed? */
    ros::Time deadline;  /* Can be used for actions that should execute for some duration */
} LinePlan;

/* changes angle to be in range [0, 2*pi] */
float normalize_angle(float x) {
    while (x > 2 * M_PI) {
        x -= 2 * M_PI;
    }
    while (x < 0) {
        x += 2 * M_PI;
    }
    return x;
}

float euclidean(float x, float y) {
    return sqrt(pow(x, 2) + pow(y, 2));
}

float sign(float x) {
    if (x < 0) return -1;
    if (x == 0) return 0;
    return 1;
}


float closest_theta_adjustment(float current, float target) {
    current = normalize_angle(current);
    target = normalize_angle(target);
    if (abs(current - target) < 2 * M_PI - abs(current - target)) {
        return target - current;
    } else {
        if (current < target) {
            return -(current + 2 * M_PI - target);
        } else {
            return 2 * M_PI - current + target;
        }
    }
}

class PathController {
public:
    ros::ServiceClient path_plan_client;
    ros::Publisher motor_publisher;
    ros::Publisher path_publisher;
    PathController(ros::NodeHandle &node_handle);
    actionlib::SimpleActionServer<planner::PlannerTargetAction> server;
    LinePlan get_line_plan(float x, float y, float theta, bool ignore_theta);
    LinePlan get_line_plan(float x, float y) { return get_line_plan(x, y, 0.0, true); }
    float line_execution_speed(const LinePlan &lp);
    vector<geometry_msgs::Point32> scans;
    bool executing_plan;
    void execute_callback(const planner::PlannerTargetGoal::ConstPtr &msg);
    void position_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void publish_twist(float linear, float angular);
    void publish_path(const planner::PathPlan &srv, int current_section);
    void preempt_callback();
    void execute_plan(LinePlan &lp);
    void stop_motors();

    // Position estimates
    float x;
    float y;
    float theta;
};

PathController::PathController(ros::NodeHandle &node_handle) :
    server (
        node_handle,
        "path_executor",
        boost::bind(&PathController::execute_callback, this, _1),
        false
    )
{
    scans = vector<geometry_msgs::Point32>();
    motor_publisher = node_handle.advertise<geometry_msgs::Twist>("/motor_controller", 10);
    path_publisher = node_handle.advertise<nav_msgs::Path>("/path", 10);
    path_plan_client = node_handle.serviceClient<planner::PathPlan>("path_plan");
    path_plan_client.waitForExistence();
    executing_plan = false;
    server.start();
    cout << "server started" << endl;
}

void PathController::publish_path(const planner::PathPlan &srv, int current_section) {
    nav_msgs::Path path;
    path.header.frame_id = "map";
    geometry_msgs::PoseStamped robot_pose;
    robot_pose.pose.position.x = x;
    robot_pose.pose.position.y = y;
    path.poses.push_back(robot_pose);
    for (int i = current_section; i < srv.response.plan.points.size(); i++) {
        geometry_msgs::Point32 p = srv.response.plan.points[i];
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = p.x;
        pose.pose.position.y = p.y;
        path.poses.push_back(pose);
    }
    path_publisher.publish(path);
}

void PathController::publish_twist(float linear, float angular) {
    geometry_msgs::Twist msg;
    msg.linear.x = linear;
    msg.angular.z = angular;
    motor_publisher.publish(msg);
}

LinePlan PathController::get_line_plan(float target_x, float target_y, float target_theta, bool ignore_theta) {
    LinePlan res = (LinePlan) {
        INITIALIZED,
        .start_x = x,
        .start_y = y,
        .target_x = target_x,
        .target_y = target_y,
        .target_theta = normalize_angle(target_theta),
        .ignore_target_theta = ignore_theta,
        .timestamp = ros::Time::now(),
        .deadline = ros::Time::now()
    };
    return res;
}

const static float MAX_LINEAR_SPEED = 0.25;
float PathController::line_execution_speed(const LinePlan &lp) {
    /* Have we travelled far enough? */
    if (euclidean(lp.start_x - lp.target_x, lp.start_y - lp.target_y) - 0.04 <= euclidean(lp.start_x - x, lp.start_y - y)) {
        return -1;
    }
    float angle_error_factor = max(0.5, pow(1 - abs(closest_theta_adjustment(theta, atan2(lp.target_y - y, lp.target_x - x))) / M_PI, 4));
    const static float CLOSEST_MAX = 0.5;
    float closest_point = CLOSEST_MAX;
    for (const geometry_msgs::Point32 &p : scans) {
        /* Finding closest point in this area:
         *
         * xxxxxx   xxxxxx
         * xxxxxx ^ xxxxxx    x
         * xxxxxx | xxxxxx    ^
         * xxx o-----o xxx    |
         *     |WALL-|        |
         *     +  E  +        |
         *
         */
        if (p.x < 0) continue;
        if (abs(p.y) < 0.1) continue;
        closest_point = min(closest_point, euclidean(p.x, p.y));
    }
    float closeness_factor = 1 - (CLOSEST_MAX - closest_point) / CLOSEST_MAX;
    cout << "closeness_factor: " << closeness_factor << endl;
    /* Add logic to slow down if walls close by */
    return MAX_LINEAR_SPEED * angle_error_factor * closeness_factor;
}

void PathController::execute_plan(LinePlan &lp) {
    float theta_correction;
    const static float INITIAL_ROTATION_SPEED = 0.8;
    float time_needed;
    float forward_speed;
    switch (lp.state) {
    case INITIALIZED:
        theta_correction = closest_theta_adjustment(theta, atan2(lp.target_y - y, lp.target_x - x));
        time_needed = abs(theta_correction / INITIAL_ROTATION_SPEED);
        lp.deadline = ros::Time::now() + ros::Duration(time_needed);
        lp.state = INITIAL_ROTATION;
        cout << "Switching to state INITIAL_ROTATION" << endl;
        publish_twist(0.0, sign(theta_correction) * INITIAL_ROTATION_SPEED);
        break;
    case INITIAL_ROTATION:
        if (ros::Time::now() >= lp.deadline) {
            publish_twist(0, 0);
            lp.deadline = ros::Time::now() + ros::Duration(1.0);
            cout << "Switching to state BEFORE_LINE_EXECUTION" << endl;
            lp.state = BEFORE_LINE_EXECUTION;
        }
        break;
    case BEFORE_LINE_EXECUTION:
        if (ros::Time::now() >= lp.deadline) {
            theta_correction = closest_theta_adjustment(theta, atan2(lp.target_y - y, lp.target_x - x));
            if (abs(theta_correction) > 0.10) {
                time_needed = abs(theta_correction / INITIAL_ROTATION_SPEED);
                lp.deadline = ros::Time::now() + ros::Duration(time_needed);
                lp.state = INITIAL_ROTATION;
                cout << "Re-switching to state INITIAL_ROTATION" << endl;
                publish_twist(0.0, sign(theta_correction) * INITIAL_ROTATION_SPEED);
            } else {
                cout << "Switching to state EXECUTING_LINE" << endl;
                lp.state = EXECUTING_LINE;
            }
        }
        break;
    case EXECUTING_LINE:
        forward_speed = line_execution_speed(lp);
        theta_correction = closest_theta_adjustment(theta, atan2(lp.target_y - y, lp.target_x - x));
        if (forward_speed > 0) {
            publish_twist(forward_speed, 0.1 * theta_correction);
            //publish_twist(forward_speed, min(0.6 * theta_correction, sign(theta_correction) * 0.8));
        } else {
            lp.deadline = ros::Time::now() + ros::Duration(1.0);
            publish_twist(0, 0);
            if (lp.ignore_target_theta) {
                cout << "Switching to state BEFORE_DONE, skipping TARGET_ROTATION" << endl;
                lp.state = BEFORE_DONE;
            } else {
                cout << "Switching to state BEFORE_TARGET_ROTATION" << endl;
                lp.state = BEFORE_TARGET_ROTATION;
            }
        }
        break;
    case BEFORE_TARGET_ROTATION:
        if (ros::Time::now() >= lp.deadline) {
            theta_correction = closest_theta_adjustment(theta, lp.target_theta);
            time_needed = abs(theta_correction / INITIAL_ROTATION_SPEED);
            lp.deadline = ros::Time::now() + ros::Duration(time_needed);
            lp.state = INITIAL_ROTATION;
            publish_twist(0.0, sign(theta_correction) * INITIAL_ROTATION_SPEED);
            cout << "Switching to state TARGET_ROTATION" << endl;
            lp.state = TARGET_ROTATION;
        }
        break;
    case TARGET_ROTATION:
        if (ros::Time::now() >= lp.deadline) {
            publish_twist(0, 0);
            lp.deadline = ros::Time::now() + ros::Duration(1.0);
            cout << "Switching to state BEFORE_DONE" << endl;
            lp.state = BEFORE_DONE;
        }
        break;
    case BEFORE_DONE:
        if (ros::Time::now() >= lp.deadline) {
            cout << "Switching to state DONE" << endl;
            publish_twist(0, 0);
            lp.state = DONE;
        }
        break;
    default:
        cerr << "REACHED UNDEFINED STATE IN PLANNER" << endl;
        break;
    }
}

void PathController::laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    scans = vector<geometry_msgs::Point32>();
    geometry_msgs::Point32 point;
    for (int degree = 0; degree < 360; degree++) {
        float distance = msg->ranges[degree];
        if (distance == INF || distance < 0.10) continue;
        float alpha = M_PI * (degree + 88.5) / 180.0;
        point.x = distance * cos(alpha) + 0.08;
        point.y = distance * sin(alpha) + 0.009;
        scans.push_back(point);
    }
}

void PathController::stop_motors() {
    motor_publisher.publish(geometry_msgs::Twist());
}

void PathController::execute_callback(const planner::PlannerTargetGoal::ConstPtr &msg) {
    ros::Rate r (128);
    while (executing_plan) {
        r.sleep();
    }
    executing_plan = true;

    planner::PathPlan srv;
    srv.request.x = msg->x;
    srv.request.y = msg->y;
    /* TODO Deal with no path!!! */
    path_plan_client.call(srv);

    int n_sections = srv.response.plan.points.size();
    int current_section = 0;
    LinePlan current_line_plan = get_line_plan(
        srv.response.plan.points[0].x,
        srv.response.plan.points[0].y
    );

    /* Implement these next */
    bool replanning = false;
    bool replanning_sleep = 0.0;

    while (ros::ok()) {
        if (server.isPreemptRequested()) {
            cout << "was preempted" << endl;
            stop_motors();
            server.setAborted();
            executing_plan = false;
            return;
        }
        /* Switch instead when we have OBSTRUCTED? */
        if (current_line_plan.state != DONE) {
            execute_plan(current_line_plan);
        } else {
            current_section++;
            if (current_section >= n_sections) {
                /* executed all segments */
                break;
            } else if (current_section == n_sections - 1) {
                /* Last segment */
                current_line_plan = get_line_plan(
                    srv.response.plan.points[current_section].x,
                    srv.response.plan.points[current_section].y,
                    msg->theta,
                    false
                );
            } else {
                /* Next segment */
                current_line_plan = get_line_plan(
                    srv.response.plan.points[current_section].x,
                    srv.response.plan.points[current_section].y
                );
            }
        }
        publish_path(srv, current_section);
        r.sleep();
    }
    stop_motors();
    server.setSucceeded();
    executing_plan = false;
}

void PathController::position_callback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    x = msg->pose.position.x;
    y = msg->pose.position.y;
    tf::Quaternion q (
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w
    );
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    theta = yaw;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "path_executor");
    ros::NodeHandle node_handle;
    PathController pc (node_handle);
    ros::Subscriber new_obstacles_subscriber = node_handle.subscribe(
        "/position",
        1,
        &PathController::position_callback,
        &pc
    );
    ros::Subscriber laser_subscriber = node_handle.subscribe(
        "/scan",
        10,
        &PathController::laser_callback,
        &pc
    );
    ros::Rate rate (128);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

