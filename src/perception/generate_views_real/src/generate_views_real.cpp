#include "generate_views_real.h"

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl_ros/point_cloud.h>

// C++ General
#include <stdlib.h>     /* atoi */

// PCL
#include <pcl/filters/extract_indices.h>

// Own
#include <perception_helper/text_color.h>

std::string colorName;

std::string object;

std::string dataset = "";

std::string save_dir_training = ros::package::getPath("extract_object") + "/Data/Views";
std::string save_dir_testing = ros::package::getPath("test_classifier") + "/Data/Views";
std::string data_extension = ".pcd";

void initParams(ros::NodeHandle n) {
}

std::string getFileName() {
    int currentMax = 0;

    std::string save_dir = save_dir_training;
    if (dataset == "testing") {
        save_dir = save_dir_testing;
    }

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
    std::string save_dir = save_dir_training;
    if (dataset == "testing") {
        save_dir = save_dir_testing;
    }

    std::cout << "Got cloud (can move now)" << std::endl;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input_cloud, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    // Remove NaNs
    //std::cout << "Remove NaNs from cloud" << std::endl;
    std::vector<int> indicesTemp;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indicesTemp);

    // Make dir for this object
    //std::cout << "Make dir for object" << std::endl;
    boost::filesystem::path dir(save_dir + "/Raw/" + colorName + "/" + object);
    boost::filesystem::create_directories(dir);

    // TODO: Maybe remove all NaNs (less space required and not needed)?!

    // Save the object Point Cloud to file
    std::cout << "Saving..." << std::endl;
    pcl::io::savePCDFileBinary(save_dir + "/Raw/" + colorName + "/" + object + "/" + getFileName() + data_extension, *cloud);
    std::cout << "Saved a new raw Point Cloud of a " << colorName << " " << object << " at:" << std::endl;
    //std::cout << save_dir << "/Raw/" << colorName << "/" << object << "/" << (atoi(getFileName().c_str()) - 1) << data_extension << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "generate_views_real");
    ros::NodeHandle nh;

    initParams(nh);

    std::cout << textColor::white;

    while (dataset == "") {
        std::cout << "Type 'training' if the data is for " << textColor::green << "training" << textColor::white << std::endl;
        std::cout << "Type 'testing' if the data is for " << textColor::red << "testing" << textColor::white << std::endl;
        std::cout << "> ";
        std::getline(std::cin, dataset);
        if (dataset == "training") {
            std::cout << textColor::green;
        } else if (dataset == "testing") {
            std::cout << textColor::red;
        } else {
            std::cout << textColor::white;
            std::cout << "'" << dataset << "' is not a valid option!" << std::endl;
            dataset = "";
        }
    }

    std::cout << "Color name: ";
    std::getline(std::cin, colorName);

    std::cout << "Object type: ";
    std::getline(std::cin, object);

    std::string status = "n";
    while (ros::ok()) {
        std::cout << "\n\n" << dataset << ": Looking for " << colorName << " " << object << std::endl;
        std::cout << "Press [ENTER] to save a new point cloud" << std::endl;
        std::cout << "Type:" << std::endl;
        std::cout << "\t'q' to stop" << std::endl;
        std::cout << "\t'n' for new object" << std::endl;
        std::cout << "\t't to change between training and testing" << std::endl;
        std::cout << "> ";
        std::getline(std::cin, status);

        if (status == "q") {
            break;
        } else if (status == "n") {
            std::cout << "Color name: ";
            std::getline(std::cin, colorName);

            std::cout << "Object type: ";
            std::getline(std::cin, object);
        } else if (status == "t") {
            dataset = "";
            while (dataset == "") {
                std::cout << textColor::white;
                std::cout << "Type 'training' if the data is for " << textColor::green << "training" << textColor::white << std::endl;
                std::cout << "Type 'testing' if the data is for " << textColor::red << "testing" << textColor::white << std::endl;
                std::cout << "> ";
                std::getline(std::cin, dataset);
                if (dataset == "training") {
                    std::cout << textColor::green;
                } else if (dataset == "testing") {
                    std::cout << textColor::red;
                } else {
                    std::cout << textColor::white;
                    std::cout << "'" << dataset << "' is not a valid option!" << std::endl;
                    dataset = "";
                }
            }
        } else if (status != "") {
            std::cout << "Unknown input" << std::endl;
        } else {
            std::cout << "Waiting for Point Cloud..." << std::endl;
            sensor_msgs::PointCloud2ConstPtr cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("camera/depth_registered/points");

            pointCloudCallback(cloud);
        }
    }
}

