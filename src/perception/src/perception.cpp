// ROS
#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include "std_msgs/String.h"

#include "perception/Object.h"

// C++ General
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

// PCL General
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

// PCL Filters
#include <pcl/filters/approximate_voxel_grid.h>

// PCL Common
#include <pcl/common/transforms.h>

// Own
#include "point_cloud_helper.h"
#include "hsv_color.h"

#define PI           3.14159265358979323846  /* pi */


hsvColor red = {1, 15, 0.5, 1, 0, 1};
hsvColor blue = {100, 200, 0, 1, 0, 1};
hsvColor green = {60, 95, 0, 1, 0, 1};
hsvColor purple = {250, 360, 0, 1, 0, 1};

std::vector<hsvColor> colors; //{red, blue, green, purple};
std::vector<std::string> colorNames;

ros::Publisher pub;
ros::Publisher object_pub;
ros::Publisher espeak_pub;

void pointCloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input_cloud) {
    colors.push_back(red);
    colors.push_back(blue);
    colors.push_back(green);
    colors.push_back(purple);

    colorNames.push_back("red");
    colorNames.push_back("blue");
    colorNames.push_back("green");
    colorNames.push_back("purple");

    // Filtering input scan to increase speed of registration.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize (0.005, 0.005, 0.005);
    approximate_voxel_filter.setInputCloud (input_cloud);
    approximate_voxel_filter.filter (*filtered_cloud);

    // Transform
    const Eigen::Vector3f translation(0.0, -0.152, 0.115);

    double yaw = 0.0;
    //double roll = -30.0 * PI / 180.0;
    double roll = -42.0 * PI / 180.0;
    double pitch = 0.0;

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

    pcl::transformPointCloud(*filtered_cloud, *filtered_cloud, translation, rotation);


    // Remove NaNs
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*filtered_cloud, *filtered_cloud, indices);

    // Find the color objects
    for (int i = 0; i < colors.size(); i++) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

        *color_cloud = *filtered_cloud;

        std::cout << color_cloud->points.size() << ", " << filtered_cloud->points.size() << std::endl;

        // Filter on color
        PointCloudHelper::HSVFilter(color_cloud, color_cloud, colors[i]);


        // Remove outliers
        PointCloudHelper::removeOutliers(color_cloud, color_cloud);

        // Check if cloud is empty
        if (color_cloud->points.size() == 0) {
            continue;
        }

        // Seperate (Segmatation)
        std::vector<pcl::PointIndices> cluster_indices = PointCloudHelper::segmentation(color_cloud);
        std::cout << cluster_indices.size() << std::endl;


        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
                cluster_cloud->points.push_back(color_cloud->points[*pit]);
            }
            cluster_cloud->width = cluster_cloud->points.size();
            cluster_cloud->height = 1;
            cluster_cloud->is_dense = true;
            cluster_cloud->header = color_cloud->header; // Will fuck up RVIZ if not here!


            std::cout << color_cloud->points.size() << ", " << cluster_cloud->points.size() << std::endl;

            // Classify
            std::string objectType = PointCloudHelper::classify(cluster_cloud, colorNames[i]);

            // Filter so only top remains
            perception::Object object = PointCloudHelper::getOptimalPickupPoint(cluster_cloud);

            object.color = colorNames[i];
            object.type = 1;

            // Output
            sensor_msgs::PointCloud2 output;

            pcl::toROSMsg(*cluster_cloud, output);

            pub.publish(output);

            object_pub.publish(object);

            std_msgs::String phrase;
            phrase.data = "I see a " + object.color + " " + objectType;

            espeak_pub.publish(phrase);
        }
    }

    // Find other obstacles
    // Booby trap
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  ros::Subscriber point_sub = nh.subscribe("camera/depth_registered/points", 1, pointCloudCallback);

  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  object_pub = nh.advertise<perception::Object> ("objectPos_wheelcenter", 1);
  espeak_pub = nh.advertise<std_msgs::String> ("espeak/string", 1);

  ros::spin();


}
