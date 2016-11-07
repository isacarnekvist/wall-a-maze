#include "generate_views_real.h"


// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include "std_msgs/String.h"

// C++ General
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <map>
#include <stdlib.h>     /* atoi */
#include <string>

// PCL General
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

// PCL Filters
#include <pcl/filters/voxel_grid.h>

// Own
#include "point_cloud_helper.h"
#include "hsv_color.h"  // Fix so this does not have to be in every package?!

#include "geometry_msgs/PointStamped.h"

#define PI           3.14159265358979323846  /* pi */

hsvColor color; //{red, blue, green, purple};
std::string colorName;

std::string object;

std::string save_dir = ros::package::getPath("train_classifier") + "/Data/Views";
std::string data_extension = ".pcd";

void initParams(ros::NodeHandle n) {
    n.getParam("/training_color", colorName);
    n.getParam("/training_object", object);

    std::map<std::string, double> color_hsv;

    n.getParam("/" + colorName + "_hsv", color_hsv);

    color = { color_hsv["h_min"], color_hsv["h_max"], color_hsv["s_min"], color_hsv["s_max"], color_hsv["v_min"], color_hsv["v_max"] };
}


std::string getFileName() {
    int currentMax = 0;

    boost::filesystem::path p(save_dir + "/Real/" + object);

    for (boost::filesystem::directory_iterator i = boost::filesystem::directory_iterator(p); i != boost::filesystem::directory_iterator(); i++) {
        if (!boost::filesystem::is_directory(i->path())) {
            std::string fileName = i->path().filename().string();

            if (fileName.find(data_extension) != std::string::npos) {
                fileName = fileName.substr(0, fileName.size() - data_extension.size());

                int current = atoi(fileName.c_str());
                if (current >= currentMax) {
                    currentMax = current + 1;
                }
            }
        }
    }

    return boost::lexical_cast<std::string>(currentMax);
}

void pointCloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input_cloud) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Remove NaNs
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*input_cloud, *filtered_cloud, indices);

    // Filter on color
    PointCloudHelper::HSVFilter(filtered_cloud, filtered_cloud, color);

    // Remove outliers
    PointCloudHelper::removeOutliers(filtered_cloud, filtered_cloud);

    // Check if cloud is empty
    if (filtered_cloud->points.size() == 0) {
        return;
    }

    // Seperate (Segmatation)
    std::vector<pcl::PointIndices> cluster_indices = PointCloudHelper::segmentation(filtered_cloud);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
            cluster_cloud->points.push_back(filtered_cloud->points[*pit]);
        }
        cluster_cloud->width = cluster_cloud->points.size();
        cluster_cloud->height = 1;
        cluster_cloud->is_dense = true;
        cluster_cloud->header = filtered_cloud->header; // Will fuck up RVIZ if not here!

        // Make dir for this object
        boost::filesystem::path dir(save_dir + "/Real/" + object);
        boost::filesystem::create_directories(dir);

        // Save the object Point Cloud to file
        pcl::io::savePCDFileASCII(save_dir + "/Real/" + object + "/" + getFileName() + data_extension, *cluster_cloud);
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "generate_views_real");
    ros::NodeHandle nh;

    initParams(nh);

    ros::Subscriber point_sub = nh.subscribe("camera/depth_registered/points", 1, pointCloudCallback);

    ros::spin();    // Spin once?!
}

