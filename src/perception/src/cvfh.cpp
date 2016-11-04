#include <ros/ros.h>
#include "cvfh.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>

#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
/*
int main (int argc, char** argv) {
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPLYFile<pcl::PointXYZ>("/home/daniel/catkin_ws/src/perception/Models/Sphere.ply", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    /*
    float resx = 150;
    float resy = 150;

    std::vector<pcl::PointCloud<pcl::PointXYZ>> views_xyz;
    std::vector<Eigen::Matrix4f> poses;
    std::vector<float> entropies;

    pcl::vi
    */
/*
    sensor_msgs::PointCloud2 output;

    pcl::toROSMsg(*cloud, output);

    output.header.frame_id = "map";

    ros::Rate loop_rate(1);
    while (ros::ok()) {

        pub.publish(output);

        loop_rate.sleep();
    }

    return (0);
}
*/
