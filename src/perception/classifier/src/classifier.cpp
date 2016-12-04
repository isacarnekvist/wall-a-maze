// ROS
#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include "std_msgs/String.h"
#include <ros/package.h>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PolygonStamped.h"

// Own
#include "classifier.h"
#include <perception_helper/vfh_helper.h>
#include <perception_helper/hsv_color.h>
#include <perception_helper/point_cloud_helper.h>

#include "classifier/Object.h"
#include "classifier/Classify.h"
#include "classifier/Find_Object.h"

// C++ General
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <map>
#include <fstream>
#include <utility>      // std::pair, std::make_pair

// PCL General
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

// PCL Features

// PCL Filters
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

// PCL Common
#include <pcl/common/transforms.h>

// FLANN
#include <flann/flann.h>
#include <flann/io/hdf5.h>



#include <pcl/io/vtk_lib_io.h>

// TA BORT!
// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl_ros/point_cloud.h>

// C++ General
#include <stdlib.h>     /* atoi */

// PCL
#include <pcl/filters/extract_indices.h>


#define PI           3.14159265358979323846  /* pi */

typedef std::pair<std::string, std::vector<float> > vfh_model;

std::string training_dir;
std::string kdtree_idx_file_name = "kdtree.idx";
std::string training_data_h5_file_name = "training_data.h5";
std::string training_data_list_file_name = "training_data.list";

std::vector<hsvColor> colors;
std::map<std::string, std::vector<std::string> > objectTypes;
std::vector<std::string> colorNames;

hsvColor wallColor;
hsvColor allColor;

ros::Publisher object_pub;
ros::Publisher point_pub;
ros::Publisher obstacle_pub;

double transform_x;
double transform_y;
double transform_z;
double transform_yaw;
double transform_roll;
double transform_pitch;

double normalRadiusSearch;
bool normalizeBins;
double EPSAngle;
double maxCurv;

std::vector<float> pointSize;

double outlierMaxNeighbours, outlierStddev, clusterTolerance, minClusterSize, maxClusterSize;

double clusterTolerance_obstacles, minClusterSize_obstacles, maxClusterSize_obstacles;

double lowCertaintyLimit, highCertaintyLimit;

std::map<std::string, std::map<std::string, float> > certaintyLimits;

Eigen::Vector3f translation;
Eigen::Quaternionf rotation;

float classifyMaxDistance;

void initParams(ros::NodeHandle n) {
    n.getParam("/classifyMaxDistance", classifyMaxDistance);

    n.getParam("/outlierMaxNeighbours", outlierMaxNeighbours);
    n.getParam("/outlierStddev", outlierStddev);

    n.getParam("/clusterTolerance", clusterTolerance);
    n.getParam("/minClusterSize", minClusterSize);
    n.getParam("/maxClusterSize", maxClusterSize);

    n.getParam("/clusterTolerance", clusterTolerance_obstacles);
    n.getParam("/minClusterSize", minClusterSize_obstacles);
    n.getParam("/maxClusterSize", maxClusterSize_obstacles);

    n.getParam("/radiusSearch", normalRadiusSearch);
    n.getParam("/normalizeBins", normalizeBins);
    n.getParam("/EPSAngle", EPSAngle);
    n.getParam("/maxCurv", maxCurv);

    n.getParam("/pointSize", pointSize);

    n.getParam("/lowCertaintyLimit", lowCertaintyLimit);
    n.getParam("/highCertaintyLimit", highCertaintyLimit);

    n.getParam("/translation/x", transform_x);
    n.getParam("/translation/y", transform_y);
    n.getParam("/translation/z", transform_z);
    n.getParam("/rotation/yaw", transform_yaw);
    n.getParam("/rotation/roll", transform_roll);
    n.getParam("/rotation/pitch", transform_pitch);

    n.getParam("/color_names", colorNames);

    std::map<std::string, double> wallColorMap;
    n.getParam("wall_color_hsv", wallColorMap);
    wallColor = { wallColorMap["h_min"], wallColorMap["h_max"], wallColorMap["s_min"], wallColorMap["s_max"], wallColorMap["v_min"], wallColorMap["v_max"] };

    std::map<std::string, double> allColorMap;
    n.getParam("all_hsv", allColorMap);
    allColor = { allColorMap["h_min"], allColorMap["h_max"], allColorMap["s_min"], allColorMap["s_max"], allColorMap["v_min"], allColorMap["v_max"] };

    for (int i = 0; i < colorNames.size(); i++) {
        // Colors
        std::map<std::string, double> color_hsv;

        n.getParam("/" + colorNames[i] + "_hsv", color_hsv);

        hsvColor color = { color_hsv["h_min"], color_hsv["h_max"], color_hsv["s_min"], color_hsv["s_max"], color_hsv["v_min"], color_hsv["v_max"] };

        colors.push_back(color);


        n.getParam("/" + colorNames[i] + "_types", objectTypes[colorNames[i]]);

        // Certainties
        std::map<std::string, float> certaintyLimit;
        n.getParam("/" + colorNames[i] + "_certaintyLimit", certaintyLimit);
        certaintyLimits[colorNames[i]] = certaintyLimit;
    }

    std::string whatTypeOfData;
    n.getParam("/whatTypeOfData", whatTypeOfData);

    training_dir = ros::package::getPath("classifier") + "/Data/Training/" + whatTypeOfData;


    double transform_x;
    double transform_y;
    double transform_z;
    double transform_yaw;
    double transform_roll;
    double transform_pitch;

    n.getParam("/translation/x", transform_x);
    n.getParam("/translation/y", transform_y);
    n.getParam("/translation/z", transform_z);
    n.getParam("/rotation/yaw", transform_yaw);
    n.getParam("/rotation/roll", transform_roll);
    n.getParam("/rotation/pitch", transform_pitch);

    translation << transform_y, -transform_z, transform_x;

    double yaw = transform_yaw * PI / 180.0;
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

    Eigen::Quaternionf rotationTemp(w, x, y, z);

    rotation = rotationTemp;
}

/** \brief Load the list of file model names from an ASCII file
  * \param models the resultant list of model name
  * \param filename the input file name
  */
bool loadFileList (std::vector<vfh_model> &models, std::string color) {
  ifstream fs;
  fs.open ((training_dir + "/" + color + "/" + training_data_list_file_name).c_str());
  if (!fs.is_open () || fs.fail ())
    return (false);

  std::string line;
  while (!fs.eof ())
  {
    getline (fs, line);
    if (line.empty ())
      continue;
    vfh_model m;
    m.first = line;
    models.push_back (m);
  }
  fs.close ();
  return (true);
}

void nearestKSearch (flann::Index<flann::L1<float> > &index, const vfh_model &model,
                int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances) {
  // Query point
  flann::Matrix<float> p = flann::Matrix<float>(new float[model.second.size ()], 1, model.second.size ());
  memcpy (&p.ptr ()[0], &model.second[0], p.cols * p.rows * sizeof (float));

  indices = flann::Matrix<int>(new int[k], 1, k);
  distances = flann::Matrix<float>(new float[k], 1, k);
  index.knnSearch (p, indices, distances, k, flann::SearchParams (512));
  delete[] p.ptr ();
}

void generateHistogram(vfh_model & vfh, pcl::PointCloud<pcl::VFHSignature308>::Ptr & point) {
    pcl::PCLPointCloud2 cloud;

    pcl::toPCLPointCloud2(*point, cloud);

    vfh.second.resize(308);

    std::vector<pcl::PCLPointField> fields;
    pcl::getFieldIndex(*point, "vhf", fields);
    for (size_t k = 0; k < fields[pcl::getFieldIndex(cloud, "vfh")].count; k++) {
        vfh.second[k] = point->points[0].histogram[k];
    }
    vfh.first = "To be classified"; // Maybe something else so we know the pose?
}

void generateHistogram(vfh_model & vfh, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud) {
    pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308>);

    VFHHelper::computeVFHSignature(vfhs, cloud, normalRadiusSearch, normalizeBins, EPSAngle, maxCurv);

    //pcl::io::savePCDFileASCII(base_dir + "/Views/Real/1" + data_extension, *vfhs);
    //pcl::io::savePCDFileASCII(data_dir + "/" + "Test" + "/" + "Test" + data_extension, *vfhs);

    generateHistogram(vfh, vfhs);
}

void classify(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, std::vector<std::pair<std::string, float> > & candidates, std::string color) {


    // Place the object in the center
    /*
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    centroid *= 1;
    std::cout << centroid << std::endl;
    pcl::demeanPointCloud<pcl::PointXYZ> (*cloud, centroid, *cloud);

    cloud->height = 1;
    cloud->width = cloud->points.size();
    */
    //pcl::io::savePCDFileASCII(base_dir + "/Views/Real/0" + data_extension, *cloud);



    int k = 83; // Todo: set to something good

    std::vector<vfh_model> models;
    flann::Matrix<int> k_indices;
    flann::Matrix<float> k_distances;
    flann::Matrix<float> data;

    loadFileList(models, color);
    flann::load_from_file(data, training_dir + "/" + color + "/" + training_data_h5_file_name, "training_data");

    vfh_model histogram;
    //loadVFHModel(histogram, "Cube", "5"); // TODO

    generateHistogram(histogram, cloud);



    flann::Index<flann::L1<float> > index (data, flann::SavedIndexParams(training_dir + "/" + color + "/" + kdtree_idx_file_name));
    index.buildIndex();
    k = index.size(); // TODO: is this good?!
    nearestKSearch(index, histogram, k, k_indices, k_distances);

    for (int i = 0; i < k; i++) {
        //std::cout << i << " - " << models.at(k_indices[0][i]).first << " (" << k_indices[0][i] << ")" << " with a distance of: " << k_distances[0][i] << std::endl;
    }

    for (size_t i = 0; i < k; i++) {
        candidates.push_back(std::make_pair(models.at(k_indices[0][i]).first, k_distances[0][i]));
    }


    //return candidates;
}

float getObjectDistance(pcl_rgb::Ptr cloud_in) {
    pcl_rgb::Ptr cloud_transformed (new pcl_rgb);
    pcl::transformPointCloud(*cloud_in, *cloud_transformed, translation, rotation);

    pcl::PointXYZRGB min_p, max_p;

    pcl::getMinMax3D(*cloud_transformed, min_p, max_p);

    return max_p.z;
}

std::pair<std::string, float> classify(pcl_rgb::Ptr cloud_in, std::string color) {
    if (getObjectDistance(cloud_in) > classifyMaxDistance) {
        return std::make_pair<std::string, float>("object", 9999999999);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);

    cloud_xyz->points.resize(cloud_in->points.size());
    for (size_t i = 0; i < cloud_in->points.size(); i++) {
        cloud_xyz->points[i].x = cloud_in->points[i].x;
        cloud_xyz->points[i].y = cloud_in->points[i].y;
        cloud_xyz->points[i].z = cloud_in->points[i].z;
    }

    std::vector<std::pair<std::string, float> > candidates;

    classify(cloud_xyz, candidates, "all");

    std::string object = "nope";
    size_t i = 0;

    for (; i < candidates.size(); i++) {
        /*
        if (certaintyLimits.at(color).find(candidates[i].first) != certaintyLimits.at(color).end() && candidates[i].second > certaintyLimits.at(color).at(candidates[i].first)) {
            return std::make_pair<std::string, float>("nope", candidates[i].second); // We are too unsure to even know if it is an object
        }
        */

        for (size_t j = 0; j < objectTypes[color].size(); j++) {
            if (candidates[i].first == objectTypes[color][j]) {
                if (certaintyLimits.at(color).find(objectTypes[color][j]) != certaintyLimits.at(color).end() && candidates[i].second < certaintyLimits.at(color).at(objectTypes[color][j])) {
                    object = objectTypes[color][j];
                    break;
                }
            }
        }

        if (object != "nope") {
            break;
        }
    }

    if (object == "nope") {
        i = 0;
        for (; i < candidates.size(); i++) {
            /*
            if (certaintyLimits.at(color).find(candidates[i].first) != certaintyLimits.at(color).end() && candidates[i].second > certaintyLimits.at(color).at(candidates[i].first)) {
                return std::make_pair<std::string, float>("nope", candidates[i].second); // We are too unsure to even know if it is an object
            }
            */

            for (size_t j = 0; j < objectTypes[color].size(); j++) {
                if (candidates[i].first == objectTypes[color][j]) {
                    if (certaintyLimits.at(color).find(objectTypes[color][j]) != certaintyLimits.at(color).end() && candidates[i].second < certaintyLimits.at(color).at(objectTypes[color][j]) + 500) {
                        object = "object";
                        break;
                    }
                }
            }

            if (object != "nope") {
                break;
            }
        }
    }

    if (object == "nope") {
        return std::make_pair<std::string, float>("nope", candidates[i].second); // We are too unsure to even know if it is an object
    }

    /*
    if (candidates[i].second > certaintyLimits.at(color).at(candidates[i].first)) {
        object = "object";    // Too unsure
    }
    */

    return std::make_pair<std::string, float>(object, candidates[i].second);
}

classifier::Object getOptimalPickupPoint(pcl_rgb::Ptr & cloud_in) {
    pcl::PointXYZRGB min_p, max_p;

    pcl::getMinMax3D(*cloud_in, min_p, max_p);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud_in);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (min_p.y - 0.015, min_p.y + 0.015);
    pass.filter(*cloud_final);

    // Calculate optimal pick up position
    // PCL -> RIKTIG
    // z = x
    // x = y

    double forward = 0.0;
    double side = 0.0;
    double height = 0.0;

    double numPoints = 0.0;
    for (pcl::PointCloud<pcl::PointXYZRGB>::iterator point = cloud_final->points.begin(); point < cloud_final->points.end(); point++) {
        numPoints++;

        forward += point->z;
        side += point->x;
        height -= point->y; // Inverted
    }

    // std::cout << "Forward: " << forward / numPoints << "\tSide: " << side / numPoints << "\tHeight: " << height / numPoints << std::endl;

    classifier::Object point;
    point.x = forward / numPoints;
    point.y = side / numPoints;
    point.z = height / numPoints;

    return point;
}

void findObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & input_cloud, std::vector<foundObject> & foundObjects) {
    std::vector<pcl_rgb::Ptr> objects = PointCloudHelper::getObjects(input_cloud, allColor, outlierMaxNeighbours, outlierStddev, clusterTolerance, minClusterSize, maxClusterSize);

    for (size_t j = 0; j < objects.size(); j++) {
        std::string color = PointCloudHelper::getDominateColor(objects[j], colors, colorNames);
        if (color == "") {
            continue;
        }

        // Classify
        std::pair<std::string, float> objectType = classify(objects[j], color);
        if (objectType.first == "nope") {
            continue;
        }

        //if (objectType.first == "star" && colorNames[i].substr(0, colorNames[i].find("_")) == "orange") {
        //    objectType.first = "patric";
        //}
        //std::cout << objectType << std::endl;

        // Filter so only top remains
        // TODO: Do this before classify?!
        pcl::transformPointCloud(*objects[j], *objects[j], translation, rotation);
        pcl::PointXYZ optimalPickupPoint = PointCloudHelper::getOptimalPickupPoint(objects[j], objectType.first);

        classifier::Object object;
        object.x = optimalPickupPoint.x;
        object.y = optimalPickupPoint.y;
        object.z = optimalPickupPoint.z;

        foundObject fObject;
        fObject.color = color.substr(0, color.find("_"));
        fObject.type = objectType.first;
        fObject.certainty = objectType.second;
        fObject.x = object.x;
        fObject.y = -object.y;
        fObject.z = object.z;

        foundObjects.push_back(fObject);
    }
}

bool classifyService(classifier::Find_Object::Request & req, classifier::Find_Object::Response & res) {
    int numTries = 1;	
    for (int loops = 0; loops < numTries; loops++) {
        // Check if there is a point cloud included
        if (req.cloud.data.size() == 0) {
            int numTries = 3;   // No point cloud included so we will try multiple times.
            sensor_msgs::PointCloud2ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("camera/depth_registered/points", ros::Duration(0.3));

            if (msg == NULL) {
                return false;
            }

            req.cloud = *msg;
        }

        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(req.cloud, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

        PointCloudHelper::preProcess(cloud, translation, rotation, pointSize, wallColor);

        std::vector<foundObject> objects;
        findObjects(cloud, objects);

        std::vector<foundObject>::iterator it = objects.begin();
        while (it != objects.end()) {
            bool keep = false;

            if (req.color.size() != 0) {
                for (size_t i = 0; i < req.color.size(); i++) {
                    if (req.type.size() != 0) {
                        for (size_t j = 0; j < req.type.size(); j++) {
                            if (req.color[i] == it->color && req.type[j] == it->type) {
                                keep = true;
                                break;
                            }
                        }
                        if (keep) {
                            break;
                        }
                    } else {
                        if (req.color[i] == it->color) {
                            keep = true;
                            break;
                        }
                    }
                }
            } else if (req.type.size() != 0){
                for (size_t i = 0; i < req.type.size(); i++) {
                    if (req.type[i] == it->type) {
                        keep = true;
                        break;
                    }
                }
            } else {
                keep = true;
            }

            if (!keep) {
                it = objects.erase(it);
            } else {
                it++;
            }
        }

        for (size_t i = 0; i < objects.size(); i++) {
            res.color.push_back(objects[i].color);
            res.type.push_back(objects[i].type);
            res.certainty.push_back(objects[i].certainty);
            res.x.push_back(objects[i].x);
            res.y.push_back(objects[i].y);
            res.z.push_back(objects[i].z);
        }

        if (res.x.size() > 0) {
            // We have found the object(s)!
            break;
        }
    }

    res.header.frame_id = "wheel_center";
    res.header.stamp = ros::Time();

    return true;
}

std::string getFileName(std::string colorName) {
    int currentMax = 0;

    std::string save_dir = ros::package::getPath("test_classifier") + "/Data/Maze/" + colorName + "/nothing";

    boost::filesystem::path p(save_dir);

    for (boost::filesystem::directory_iterator i = boost::filesystem::directory_iterator(p); i != boost::filesystem::directory_iterator(); i++) {
        if (!boost::filesystem::is_directory(i->path())) {
            std::string fileName = i->path().filename().string();

            if (fileName.find(".pcd") != std::string::npos) {
                fileName = fileName.substr(0, fileName.size() - 4);

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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    *cloud = *input_cloud;

    PointCloudHelper::preProcess(cloud, translation, rotation, pointSize, wallColor);

    pcl_rgb::Ptr cloud_object (new pcl_rgb);
    *cloud_object = *cloud;

    //pcl::io::savePCDFileBinary(training_dir + "/test.pcd", *cloud);

    // Get objects
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> objects = PointCloudHelper::getObjects(cloud_object, allColor, outlierMaxNeighbours, outlierStddev, clusterTolerance, minClusterSize, maxClusterSize);

    for (size_t j = 0; j < objects.size(); j++) {
        std::string color = PointCloudHelper::getDominateColor(objects[j], colors, colorNames);
        if (color == "") {
            continue;
        }

        // Classify
        // Type, certainty
        std::pair<std::string, float> objectType = classify(objects[j], color);
        if (objectType.first == "nope") {
            // Could not classify this object!
            //std::cout << objectType.second << std::endl;
            continue;
        }

        pcl::transformPointCloud(*objects[j], *objects[j], translation, rotation);
        pcl::PointXYZ optimalPickupPoint = PointCloudHelper::getOptimalPickupPoint(objects[j], objectType.first);

        if (objectType.first == "star" && color.substr(0, color.find("_")) == "orange") {
            objectType.first = "patric";
        }

        classifier::Object object;
        object.x = optimalPickupPoint.x;
        object.y = optimalPickupPoint.y;
        object.z = optimalPickupPoint.z;

        std::string temp;
        if (objectType.first != "patric") {
            temp = color.substr(0, color.find("_")) + " ";
            std::cout << color.substr(0, color.find("_")) << " ";
        }
        //detectedObjects.push_back(temp + objectType.first);
        std::cout << objectType.first << " (" << objectType.second << ") at: " << "X: " << object.x << ", Y: " << object.y << ", Z: " << object.z <<  std::endl;


        object.color = color.substr(0, color.find("_"));
        object.type = objectType.first;


        object_pub.publish(object);

        //boost::filesystem::path dir(ros::package::getPath("test_classifier") + "/Data/Maze/" + object.color + "/nothing");
        //boost::filesystem::create_directories(dir);
        //pcl::io::savePCDFileBinary(ros::package::getPath("test_classifier") + "/Data/Maze/" + object.color + "/nothing/" + getFileName(object.color) + ".pcd", *input_cloud);
    }

    // Obstacles
    if (cloud->size() == 0) {
        return;
    }

    // Transform
    pcl::transformPointCloud(*cloud, *cloud, translation, rotation);

    for (size_t i = 0; i < colors.size(); i++) {
        pcl_rgb::Ptr temp (new pcl_rgb);
        PointCloudHelper::HSVFilter(cloud, temp, colors[i]);
    }

    if (cloud->size() == 0) {
        return;
    }

    // TODO: maybe different values for these objects since they are larger
    // Segmentation
    objects = PointCloudHelper::segmentation(cloud, clusterTolerance_obstacles, minClusterSize_obstacles, maxClusterSize_obstacles);

    for (size_t i = 0; i < objects.size(); i++) {
        pcl::PointXYZRGB min_p, max_p;

        pcl::getMinMax3D(*objects[i], min_p, max_p);

        geometry_msgs::PolygonStamped poly;

        geometry_msgs::Point32 point[2];

        point[0].x = -1;
        point[0].y = min_p.x;
        point[1].x = -1;
        point[1].y = max_p.x;

        if (false) {
            // Naive
            point[0].x = min_p.z;
            point[1].x = max_p.z;
        } else {
            for (size_t p = 0; p < objects[i]->size(); p++) {
                if (point[0].x < 0 && objects[i]->points[p].x == min_p.x) {
                    point[0].x = objects[i]->points[p].z;
                }
                if (point[1].x < 0 && objects[i]->points[p].x == max_p.x) {
                    point[1].x = objects[i]->points[p].z;
                }

                if (point[0].x >= 0 && point[1].x >= 0) {
                    break;
                }
            }
        }

        poly.polygon.points.push_back(point[0]);
        poly.polygon.points.push_back(point[1]);

        // Publish x = z and y = x
        obstacle_pub.publish(poly);
    }
}


void pointCloudCallback2(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input_cloud) {
    // Filtering input scan to increase speed of registration.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    PointCloudHelper::resizePoints(input_cloud, filtered_cloud, pointSize[0], pointSize[1], pointSize[2]);

    // Transform
    //pcl::transformPointCloud(*filtered_cloud, *filtered_cloud, translation, rotation);

    std::vector<std::string> detectedObjects;
    // Find the color objects
    for (int i = 0; i < colors.size(); i++) {
        std::vector<pcl_rgb::Ptr> objects = PointCloudHelper::getObjects(filtered_cloud, colors[i], outlierMaxNeighbours, outlierStddev, clusterTolerance, minClusterSize, maxClusterSize);

        for (size_t j = 0; j < objects.size(); j++) {
            // Classify
            std::pair<std::string, float> objectType = classify(objects[j], colorNames[i]);
            if (objectType.first == "nope") {
                continue;
            }
            if (objectType.first == "star" && colorNames[i].substr(0, colorNames[i].find("_")) == "orange") {
                objectType.first = "patric";
            }
            //std::cout << objectType << std::endl;

            // Filter so only top remains
            // TODO: Do this before classify?!
            pcl::transformPointCloud(*objects[j], *objects[j], translation, rotation);
            classifier::Object object = getOptimalPickupPoint(objects[j]);

            object.y = -object.y;
	    std::string temp;
            if (objectType.first != "patric") {
		temp = colorNames[i].substr(0, colorNames[i].find("_")) + " ";
                std::cout << colorNames[i].substr(0, colorNames[i].find("_")) << " ";
            }
	    detectedObjects.push_back(temp + objectType.first);
            std::cout << objectType.first << " (" << objectType.second << ") at: " << "X: " << object.x << ", Y: " << object.y << ", Z: " << object.z <<  std::endl;


            object.color = colorNames[i].substr(0, colorNames[i].find("_"));
            object.type = objectType.first;

            // Output

            geometry_msgs::PointStamped point;
            point.point.x = object.x;
            point.point.y = object.y;
            point.point.z = object.z;
            point.header.frame_id = "wheel_center";
            point.header.stamp = ros::Time();

            object_pub.publish(object);
            point_pub.publish(point);
        }
    }

    std::string output = "[ ";
    for (size_t i = 0; i < detectedObjects.size(); i++) {
	output += detectedObjects[i];
	if (i < detectedObjects.size() - 1) {
		output += ", ";
	}
    }
    output += " ]";
    if (detectedObjects.size() != 0) {
	//std::cout << output << std::endl;
    }

    // Find other obstacles
    // Booby trap
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "classifier");
    ros::NodeHandle nh;

    initParams(nh);

    ros::Subscriber point_sub = nh.subscribe("camera/depth_registered/points", 1, pointCloudCallback);

    object_pub = nh.advertise<classifier::Object> ("objectPos_wheelcenter2", 1);
    //point_pub = nh.advertise<geometry_msgs::PointStamped> ("objectPos_wheelcenter", 1);
    obstacle_pub = nh.advertise<geometry_msgs::PolygonStamped> ("obstaclePos_wheelcenter", 1);

    ros::ServiceServer service = nh.advertiseService("find_object", classifyService);

    // Test if this work!
    ros::Rate loop_rate(10.0);

    while (ros::ok()) {
        ros::spinOnce();

        loop_rate.sleep();
    }
}
