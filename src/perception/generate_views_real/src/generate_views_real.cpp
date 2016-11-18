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
#include <perception_helper/point_cloud_helper.h>
#include <perception_helper/hsv_color.h>

#include "geometry_msgs/PointStamped.h"

#define PI           3.14159265358979323846  /* pi */

hsvColor color; //{red, blue, green, purple};
std::string colorName;

std::string object;

std::string save_dir = ros::package::getPath("train_classifier") + "/Data/Views";
std::string data_extension = ".pcd";

double outlierMaxNeighbours, outlierStddev, clusterTolerance, minClusterSize, maxClusterSize;

void initParams(ros::NodeHandle n) {
    n.getParam("/outlierMaxNeighbours", outlierMaxNeighbours);
    n.getParam("/outlierStddev", outlierStddev);

    n.getParam("/clusterTolerance", clusterTolerance);
    n.getParam("/minClusterSize", minClusterSize);
    n.getParam("/maxClusterSize", maxClusterSize);

    std::map<std::string, double> color_hsv;

    n.getParam("/" + colorName + "_hsv", color_hsv);

    if (color_hsv.size() == 0) {
        std::cout << "Color " << colorName << " does not exist!" << std::endl;
        throw(0);
    }

    color = { color_hsv["h_min"], color_hsv["h_max"], color_hsv["s_min"], color_hsv["s_max"], color_hsv["v_min"], color_hsv["v_max"] };
}

std::string getFileName() {
    int currentMax = 0;

    boost::filesystem::path p(save_dir + "/Raw/" + colorName + "/" + object);

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

void pointCloudCallback(sensor_msgs::PointCloud2ConstPtr & input_cloud) {
    std::cout << "Got cloud" << std::endl;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input_cloud, pcl_pc2);
    pcl_rgb::Ptr cloud(new pcl_rgb);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    // Make dir for this object
    //std::cout << "Make dir for object" << std::endl;
    boost::filesystem::path dir(save_dir + "/Real/" + object);
    boost::filesystem::path dir2(save_dir + "/Raw/" + colorName + "/" + object);
    boost::filesystem::create_directories(dir);
    boost::filesystem::create_directories(dir2);

    pcl_rgb::Ptr cloud_raw (new pcl_rgb);
    *cloud_raw = *cloud;
    /*
    cloud_raw->width = cloud_raw->points.size();
    cloud_raw->height = 1;
    cloud_raw->is_dense = true;
    cloud_raw->header = cloud->header;
*/
    // Save the object Point Cloud to file
    pcl::io::savePCDFileASCII(save_dir + "/Raw/" + colorName + "/" + object + "/" + getFileName() + data_extension, *cloud_raw);
    std::cout << "Saved a new raw Point Cloud of a " << colorName << " " << object << " at:" << std::endl;
    std::cout << save_dir << "/Raw/" << colorName << "/" << object << "/" << getFileName() << data_extension << std::endl;

    //std::vector<pcl_rgb::Ptr> objects = PointCloudHelper::getObjects(cloud, color, outlierMaxNeighbours, outlierStddev, clusterTolerance, minClusterSize, maxClusterSize);

    //for (size_t i = 0; i < objects.size(); i++) {
        // Save the object Point Cloud to file
        //pcl::io::savePCDFileASCII(save_dir + "/Real/" + object + "/" + getFileName() + data_extension, *objects[i]);
        //std::cout << "Saved a new Point Cloud of a " << colorName << " " << object << " at:" << std::endl;
        //std::cout << save_dir << "/Real/" << object << "/" << getFileName() << data_extension << std::endl;
    //}
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "generate_views_real");
    ros::NodeHandle nh;

    std::cout << "Color name: ";
    std::getline(std::cin, colorName);

    initParams(nh);

    std::cout << "Object type: ";
    std::getline(std::cin, object);

    std::string status;
    while (ros::ok()) {
        std::cout << "Press [ENTER] to save a new point cloud, write 'n' to stop: ";
        std::getline(std::cin, status);

        if (status == "n") {
            break;
        }

        sensor_msgs::PointCloud2ConstPtr cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("camera/depth_registered/points");

        pointCloudCallback(cloud);
    }
}

