#include "extract_object.h"

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

std::string load_dir = ros::package::getPath("extract_object") + "/Data/Views/Raw";
std::string save_dir = ros::package::getPath("train_classifier") + "/Data/Views/Real";
std::string data_extension = ".pcd";

double outlierMaxNeighbours, outlierStddev, clusterTolerance, minClusterSize, maxClusterSize;

void initParams(ros::NodeHandle n) {
    n.getParam("/outlierMaxNeighbours", outlierMaxNeighbours);
    n.getParam("/outlierStddev", outlierStddev);

    n.getParam("/clusterTolerance", clusterTolerance);
    n.getParam("/minClusterSize", minClusterSize);
    n.getParam("/maxClusterSize", maxClusterSize);
}

std::string getFileName(std::string object) {
    int currentMax = 0;

    boost::filesystem::path p(save_dir + "/" + object);

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

std::vector<std::pair<std::pair<std::string, std::string>, std::string> > getObjectLocations() {
    std::vector<std::pair<std::pair<std::string, std::string>, std::string> > objectLocations;

    boost::filesystem::path p(load_dir);

    for (boost::filesystem::directory_iterator i = boost::filesystem::directory_iterator(p); i != boost::filesystem::directory_iterator(); i++) {
        std::string color = i->path().filename().string();
        if (boost::filesystem::is_directory(i->path())) {
            for (boost::filesystem::directory_iterator j = boost::filesystem::directory_iterator(i->path()); j != boost::filesystem::directory_iterator(); j++) {
                std::string object = j->path().filename().string();
                if (boost::filesystem::is_directory(j->path())) {
                    for (boost::filesystem::directory_iterator k = boost::filesystem::directory_iterator(j->path()); k != boost::filesystem::directory_iterator(); k++) {
                        if (!boost::filesystem::is_directory(k->path())) {
                            std::string fileName = k->path().filename().string();

                            bool found = true;
                            for (size_t l = 1; l <= data_extension.size(); l++) {
                                if (fileName[fileName.size() - l] != data_extension[data_extension.size() - l]) {
                                    found = false;
                                    break;
                                }
                            }

                            if (found) {
                                std::pair<std::string, std::string> type = std::make_pair<std::string, std::string>(color, object);
                                objectLocations.push_back(std::make_pair<std::pair<std::string, std::string>, std::string>(type, k->path().string()));
                            }
                        }
                    }
                }
            }
        }
    }

    return objectLocations;
}

void extract(pcl_rgb::Ptr & cloud, std::string colorName, std::string object, ros::NodeHandle n) {
    // Make dir for this object
    //std::cout << "Make dir for object" << std::endl;
    boost::filesystem::path dir(save_dir + "/" + object);
    boost::filesystem::create_directories(dir);

    std::map<std::string, double> color_hsv;

    n.getParam("/" + colorName + "_hsv", color_hsv);

    if (color_hsv.size() == 0) {
        std::cout << "Color " << colorName << " does not exist!" << std::endl;
        throw(0);
    }

    hsvColor color = { color_hsv["h_min"], color_hsv["h_max"], color_hsv["s_min"], color_hsv["s_max"], color_hsv["v_min"], color_hsv["v_max"] };

    std::vector<pcl_rgb::Ptr> objects = PointCloudHelper::getObjects(cloud, color, outlierMaxNeighbours, outlierStddev, clusterTolerance, minClusterSize, maxClusterSize);

    std::cout << objects.size() << std::endl;
    for (size_t i = 0; i < objects.size(); i++) {
        // Save the object Point Cloud to file
        pcl::io::savePCDFileBinary(save_dir + "/" + object + "/" + getFileName(object) + data_extension, *objects[i]);
        std::cout << "Saved a new Point Cloud of a " << colorName << " " << object << " at:" << std::endl;
        std::cout << save_dir << "/" << object << "/" << (atoi(getFileName(object).c_str()) - 1) << data_extension << std::endl;
    }
}

void extract_object(ros::NodeHandle n) {
    // Get object locations
    std::cout << "Getting locations of objects" << std::endl;
    std::vector<std::pair<std::pair<std::string, std::string>, std::string> > objectLocations = getObjectLocations();

    for (size_t i = 0; i < objectLocations.size(); i++) {
        std::cout << "Processing view " << (i+1) << " of " << objectLocations.size() << std::endl;
        std::cout << "A " << objectLocations[i].first.first << " " << objectLocations[i].first.second << std::endl;

        pcl_rgb::Ptr cloud (new pcl_rgb);
        pcl::io::loadPCDFile<pcl::PointXYZRGB>(objectLocations[i].second, *cloud);

        extract(cloud, objectLocations[i].first.first, objectLocations[i].first.second, n);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "extract_object");
    ros::NodeHandle nh;

    initParams(nh);

    extract_object(nh);
}




int getNumFilesWithExtensionMaxDepth(std::string dir, std::string extension, int depth) {
    if (depth < 0) {
        return 0;
    }
    depth--;

    int numObjects = 0;

    boost::filesystem::path p(dir);

    for (boost::filesystem::directory_iterator i = boost::filesystem::directory_iterator(p); i != boost::filesystem::directory_iterator(); i++) {
        if (!boost::filesystem::is_directory(i->path())) {
            std::string fileName = i->path().filename().string();

            bool found = true;
            for (size_t i = 1; i <= extension.size(); i++) {
                if (fileName[fileName.size() - i] != extension[extension.size() - i]) {
                    found = false;
                    break;
                }
            }

            if (found) {
                numObjects++;
            }
        } else {
            numObjects += getNumFilesWithExtensionMaxDepth(i->path().string(), extension, depth);
        }
    }

    return numObjects;
}

