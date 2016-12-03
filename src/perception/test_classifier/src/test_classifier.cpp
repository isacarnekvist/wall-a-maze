#include "test_classifier.h"

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
// Writing file
#include <iostream>
#include <fstream>

// PCL General
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

// PCL Filters
#include <pcl/filters/voxel_grid.h>

// Own
#include <perception_helper/point_cloud_helper.h>
#include <perception_helper/hsv_color.h>
#include "classifier/Find_Object.h"
#include "objectTypeAndLocation.h"
#include <perception_helper/text_color.h>

#include "geometry_msgs/PointStamped.h"

#define PI           3.14159265358979323846  /* pi */

std::string load_dir = ros::package::getPath("test_classifier") + "/Data/Views/Raw";
std::string save_dir = ros::package::getPath("test_classifier") + "/Data/Report";
std::string data_extension = ".pcd";
std::string report_extension = ".txt";

double waitTime;

void initParams(ros::NodeHandle n) {
    n.getParam("/waitTime", waitTime);
}

std::string getFileName() {
    boost::filesystem::path dir(save_dir);
    boost::filesystem::create_directories(dir);

    int currentMax = 0;

    boost::filesystem::path p(save_dir);

    for (boost::filesystem::directory_iterator i = boost::filesystem::directory_iterator(p); i != boost::filesystem::directory_iterator(); i++) {
        if (!boost::filesystem::is_directory(i->path())) {
            std::string fileName = i->path().filename().string();

            if (fileName.find(report_extension) != std::string::npos) {
                fileName = fileName.substr(0, fileName.size() - report_extension.size());

                int current = atoi(fileName.c_str());
                if (current >= currentMax) {
                    currentMax = current + 1;
                }
            }
        }
    }

    return boost::lexical_cast<std::string>(currentMax);
}

std::vector<objectTypeAndLocation> getObjectLocations() {
    std::vector<objectTypeAndLocation> objectLocations;

    boost::filesystem::path p(load_dir);

    for (boost::filesystem::directory_iterator i = boost::filesystem::directory_iterator(p); i != boost::filesystem::directory_iterator(); i++) {
        std::string color = i->path().filename().string();
        if (boost::filesystem::is_directory(i->path())) {
            for (boost::filesystem::directory_iterator j = boost::filesystem::directory_iterator(i->path()); j != boost::filesystem::directory_iterator(); j++) {
                std::string type = j->path().filename().string();
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
                                objectTypeAndLocation object;
                                object.color = color;
                                object.type = type;
                                object.location = k->path().string();
                                objectLocations.push_back(object);
                            }
                        }
                    }
                }
            }
        }
    }

    return objectLocations;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_classifier");
    ros::NodeHandle nh;

    initParams(nh);

    std::cout << textColor::white;

    ros::ServiceClient client = nh.serviceClient<classifier::Find_Object>("find_object");

    std::vector<objectTypeAndLocation> objectLocations = getObjectLocations();

    int right = 0;
    int wrong = 0;
    int none = 0;
    int error = 0;
    std::vector<objectTypeAndLocation> wronglyClassified;
    std::vector<std::vector<objectTypeAndLocation> > classifiedAs;
    for (size_t i = 0; i < objectLocations.size() && ros::ok(); i++) {
        std::cout << "Testing " << (i+1) << " of " << objectLocations.size() << std::endl;

        pcl_rgb::Ptr cloud (new pcl_rgb);
        pcl::io::loadPCDFile<pcl::PointXYZRGB>(objectLocations[i].location, *cloud);

        sensor_msgs::PointCloud2 cloud_ros;
        pcl::toROSMsg(*cloud, cloud_ros);

        classifier::Find_Object srv;
        srv.request.cloud = cloud_ros;

        if (client.call(srv)) {
            if (srv.response.color.size() == 0) {
                if (objectLocations[i].type == "nothing") {
                    right++;
                    std::cout << textColor::green << "Correctly not classified: " << objectLocations[i].color << " " << objectLocations[i].type << textColor::white << std::endl;
                    continue;
                }
                none++;
                std::cout << textColor::yellow << "No classification recieved for: " << objectLocations[i].color << " " << objectLocations[i].type << textColor::white << std::endl;
                wronglyClassified.push_back(objectLocations[i]);
                objectTypeAndLocation ob;
                ob.color = "";
                ob.type = "";
                ob.certainty = -1;
                std::vector<objectTypeAndLocation > classification;
                classification.push_back(ob);
                classifiedAs.push_back(classification);
            } else if (srv.response.color.size() == 1) {
                if (srv.response.color[0] == objectLocations[i].color && srv.response.type[0] == objectLocations[i].type) {
                    right++;
                    std::cout << textColor::green << "Correctly classified: " << srv.response.color[0] << " " << srv.response.type[0] << " (" << srv.response.certainty[0] << ")" << textColor::white << std::endl;
                } else {
                    wrong++;
                    std::cout << textColor::red << "When it was:\n\t" << objectLocations[i].color << " " << objectLocations[i].type << std::endl;
                    std::cout << "It classified it as:\n\t" << srv.response.color[0] << " " << srv.response.type[0] << " (" << srv.response.certainty[0] << ")" << textColor::white << std::endl;
                    wronglyClassified.push_back(objectLocations[i]);
                    objectTypeAndLocation ob;
                    ob.color = srv.response.color[0];
                    ob.type = srv.response.type[0];
                    ob.certainty = srv.response.certainty[0];
                    std::vector<objectTypeAndLocation> classification;
                    classification.push_back(ob);
                    classifiedAs.push_back(classification);
                }
            } else {
                wrong++;
                std::cout << textColor::red << "When it was:\n\t" << objectLocations[i].color << " " << objectLocations[i].type << std::endl;
                std::cout << "It classified it as:"  << std::endl;
                for (size_t j = 0; j < srv.response.color.size(); j++) {
                    std::cout << "\t" << srv.response.color[j] << " " << srv.response.type[j] << " (" << srv.response.certainty[j] << ")" << std::endl;
                }
                std::cout << textColor::white;
                std::vector<objectTypeAndLocation> classification;

                for (size_t j = 0; j < srv.response.color.size(); j++) {
                    objectTypeAndLocation ob;
                    ob.color = srv.response.color[j];
                    ob.type = srv.response.type[j];
                    ob.certainty = srv.response.certainty[j];
                    classification.push_back(ob);
                }
                wronglyClassified.push_back(objectLocations[i]);
                classifiedAs.push_back(classification);
            }
        } else {
            error++;
            std::cout << textColor::cyan << "Some error..." << textColor::white << std::endl;
        }
        std::cout << std::endl;
    }

    std::cout << "Classified " << right << " of " << objectLocations.size() << " (" << ((((float)right)/((float)objectLocations.size())) * 100) << "%) correctly" << std::endl;
    std::cout << "Did not classify " << none << " of " << objectLocations.size() << " (" << ((((float)none)/((float)objectLocations.size())) * 100) << "%)" << std::endl;
    std::cout << "Classified " << wrong << " of " << objectLocations.size() << " (" << ((((float)wrong)/((float)objectLocations.size())) * 100) << "%) incorrectly" << std::endl;
    std::cout << "Error on " << error << " of " << objectLocations.size() << " (" << ((((float)error)/((float)objectLocations.size())) * 100) << "%)" << std::endl;

    std::string fileName = getFileName();

    std::ofstream myfile;
    myfile.open((save_dir + "/" + fileName + report_extension).c_str(), std::ios::out | std::ios::trunc);
    if (myfile.is_open()) {
    myfile << "Classified " << right << " of " << objectLocations.size() << " (" << ((((float)right)/((float)objectLocations.size())) * 100) << "%) correctly\n";
    myfile << "Did not classify " << none << " of " << objectLocations.size() << " (" << ((((float)none)/((float)objectLocations.size())) * 100) << "%)\n";
    myfile << "Classified " << wrong << " of " << objectLocations.size() << " (" << ((((float)wrong)/((float)objectLocations.size())) * 100) << "%) incorrectly\n";
    myfile << "Error on " << error << " of " << objectLocations.size() << " (" << ((((float)error)/((float)objectLocations.size())) * 100) << "%)\n\n";
    myfile << "Classified these wrong:\n\n";

    for (size_t i = 0; i < wronglyClassified.size(); i++) {
        myfile << "------------\nWhen it was:\n\t" << wronglyClassified[i].color << " " << wronglyClassified[i].type << "\n";
        myfile << "It classified as:\n";
        for (size_t j = 0; j < classifiedAs[i].size(); j++) {
            if (classifiedAs[i][j].certainty != -1) {
                myfile << "\t" << classifiedAs[i][j].color << " " << classifiedAs[i][j].type << " (" << classifiedAs[i][j].certainty << ")\n";
            }
        }
        myfile << "Cloud location:\n\t" << wronglyClassified[i].location << "\n------------\n\n";
    }

    myfile.close();

    std::cout << "Check file for more information:" << std::endl << save_dir << "/" << fileName << report_extension << std::endl;
    } else {
      std::cout << "Unable to save report" << std::endl;
    }
}
