#include "calibrate.h"

// ROS
#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>

// PCL Common
#include <pcl/common/transforms.h>


#define PI           3.14159265358979323846  /* pi */

ros::Publisher pub;

double transform_x;
double transform_y;
double transform_z;
double transform_yaw;
double transform_roll;
double transform_pitch;

void initParams(ros::NodeHandle n) {
    n.getParam("/translation/x", transform_x);
    n.getParam("/translation/y", transform_y);
    n.getParam("/translation/z", transform_z);
    n.getParam("/rotation/yaw", transform_yaw);
    n.getParam("/rotation/roll", transform_roll);
    n.getParam("/rotation/pitch", transform_pitch);
}

void pointCloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input_cloud) {
    // Transform
    const Eigen::Vector3f translation(transform_y, -transform_z, transform_x);

    double yaw = transform_yaw * PI / 180.0;
    //double roll = -30.0 * PI / 180.0;
    //double roll = -42.0 * PI / 180.0;
    double roll = -transform_roll * PI / 180.0;
    double pitch = transform_pitch * PI / 180.0;

    double t0 = std::cos(yaw * 0.5f);
    double t1 = std::sin(yaw * 0.5f);
    double t2 = std::cos(roll * 0.5f);
    double t3 = std::sin(roll * 0.5f);
    double t4 = std::cos(pitch * 0.5f);
    double t5 = std::sin(pitch * 0.5f);

    double w = t0 * t2 * t4 + t1 * t3 * t5;
    double x = t0 * t3 * t4 - t1 * t2 * t5;
    double y = t0 * t2 * t5 + t1 * t3 * t4;
    double z = t1 * t2 * t4 - t0 * t3 * t5;

    //std::cout << "w: " << w << "\tx: " << x << "\ty: " << y << "\tz: " << z << std::endl;

    Eigen::Quaternionf rotation(w, x, y, z);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::transformPointCloud(*input_cloud, *filtered_cloud, translation, rotation);


    // Output
    sensor_msgs::PointCloud2 output;

    pcl::toROSMsg(*filtered_cloud, output);

    pub.publish(output);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "perception_calibrate");
    ros::NodeHandle nh;

    initParams(nh);

    ros::Subscriber sub = nh.subscribe("camera/depth_registered/points", 1, pointCloudCallback);

    pub = nh.advertise<sensor_msgs::PointCloud2>("calibrate", 1);

    ros::spin();
}
