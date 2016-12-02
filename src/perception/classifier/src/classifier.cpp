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

    return max_p.x;
}

std::pair<std::string, float> classify(pcl_rgb::Ptr cloud_in, std::string color) {
    if (getObjectDistance(cloud_in) > classifyMaxDistance) {
        return std::make_pair<std::string, float>("nope", 9999999999);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);

    cloud_xyz->points.resize(cloud_in->points.size());
    for (size_t i = 0; i < cloud_in->points.size(); i++) {
        cloud_xyz->points[i].x = cloud_in->points[i].x;
        cloud_xyz->points[i].y = cloud_in->points[i].y;
        cloud_xyz->points[i].z = cloud_in->points[i].z;
    }

    std::vector<std::pair<std::string, float> > candidates;

    classify(cloud_xyz, candidates, color);

    std::string object = "nope";
    size_t i = 0;
    for (; i < candidates.size(); i++) {
        if (candidates[i].second > certaintyLimits.at(color).at(candidates[i].first)) {
            return std::make_pair<std::string, float>("nope", candidates[i].second); // We are too unsure to even know if it is an object
        }

        for (size_t j = 0; j < objectTypes[color].size(); j++) {
            if (candidates[i].first == objectTypes[color][j]) {
                object = objectTypes[color][j];
                break;
            }
        }

        if (object != "nope") {
            break;
        }
    }

    if (object == "nope") {
        return std::make_pair<std::string, float>("nope", candidates[i].second); // We are too unsure to even know if it is an object
    }

    if (candidates[i].second > certaintyLimits.at(color).at(candidates[i].first)) {
        object = "object";    // Too unsure
    }

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
    std::vector<std::string> detectedObjects;
    // Find the color objects
    for (int i = 0; i < colors.size(); i++) {
        std::vector<pcl_rgb::Ptr> objects = PointCloudHelper::getObjects(input_cloud, colors[i], outlierMaxNeighbours, outlierStddev, clusterTolerance, minClusterSize, maxClusterSize);

        for (size_t j = 0; j < objects.size(); j++) {
            // Classify
            std::pair<std::string, float> objectType = classify(objects[j], colorNames[i]);
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
            classifier::Object object = getOptimalPickupPoint(objects[j]);

            foundObject fObject;
            fObject.color = colorNames[i].substr(0, colorNames[i].find("_"));
            fObject.type = objectType.first;
            fObject.certainty = objectType.second;
            fObject.x = object.x;
            fObject.y = -object.y;
            fObject.z = object.z;

            foundObjects.push_back(fObject);
        }
    }
}

bool classifyService(classifier::Find_Object::Request & req, classifier::Find_Object::Response & res) {
    // Check if there is a point cloud included
    if (req.cloud.data.size() == 0) {
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

    res.header.frame_id = "wheel_center";
    res.header.stamp = ros::Time();

    std::cout << "Cloud size: " << cloud->size() << std::endl;

    return true;
}

void pointCloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input_cloud) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    *cloud = *input_cloud;

    PointCloudHelper::preProcess(cloud, translation, rotation, pointSize, wallColor);

    pcl::io::savePCDFileBinary(training_dir + "/test.pcd", *cloud);

    // Colored objects
    for (size_t i = 0; i < colors.size(); i++) {
        // Get objects
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> objects = PointCloudHelper::getObjects(cloud, colors[i], outlierMaxNeighbours, outlierStddev, clusterTolerance, minClusterSize, maxClusterSize);

        for (size_t j = 0; j < objects.size(); j++) {
            // Classify
            // Type, certainty
            std::pair<std::string, float> objectType = classify(objects[j], colorNames[i]);
            if (objectType.first == "nope") {
                // Could not classify this object!
                std::cout << objectType.second << std::endl;
                continue;
            }

            pcl::transformPointCloud(*objects[j], *objects[j], translation, rotation);
            pcl::PointXYZ optimalPickupPoint = PointCloudHelper::getOptimalPickupPoint(objects[j], objectType.first);

            if (objectType.first == "star" && colorNames[i].substr(0, colorNames[i].find("_")) == "orange") {
                objectType.first = "patric";
            }

            classifier::Object object;
            object.x = optimalPickupPoint.x;
            object.y = optimalPickupPoint.y;
            object.z = optimalPickupPoint.z;

            std::string temp;
            if (objectType.first != "patric") {
                temp = colorNames[i].substr(0, colorNames[i].find("_")) + " ";
                std::cout << colorNames[i].substr(0, colorNames[i].find("_")) << " ";
            }
            //detectedObjects.push_back(temp + objectType.first);
            std::cout << objectType.first << " (" << objectType.second << ") at: " << "X: " << object.x << ", Y: " << object.y << ", Z: " << object.z <<  std::endl;


            object.color = colorNames[i].substr(0, colorNames[i].find("_"));
            object.type = objectType.first;


            object_pub.publish(object);
        }
    }

    // Obstacles

    // Transform
    pcl::transformPointCloud(*cloud, *cloud, translation, rotation);

    // TODO: maybe different values for these objects since they are larger
    // Segmentation
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> objects = PointCloudHelper::segmentation(cloud, clusterTolerance, minClusterSize, maxClusterSize);

    pcl::PointXYZ minX, maxX, minY, maxY;
    for (size_t i = 0; i < objects.size(); i++) {
        pcl::PointXYZRGB min_p, max_p;

        pcl::getMinMax3D(*objects[i], min_p, max_p);

        if (true) {
            // Naive
            geometry_msgs::PolygonStamped poly;

            geometry_msgs::Point32 point[2];

            point[0].x = min_p.z;
            point[0].y = min_p.x;
            point[1].x = max_p.z;
            point[1].y = max_p.x;

            poly.polygon.points.push_back(point[0]);
            poly.polygon.points.push_back(point[1]);

            // Publish x = z and y = x
            obstacle_pub.publish(poly);
        } else {
            // Advanced
            float min, max;
            float height = max_p.y - min_p.y;
            if (height > 0.02) {
                min = min_p.y + (height/2.0) - 0.005;
                max = max_p.y - (height/2.0) + 0.005;
            } else {
                // TODO: Make better
                max = max_p.y;
                min = min_p.y;
            }

            pcl::PassThrough<pcl::PointXYZRGB> pass;
            pass.setInputCloud (objects[i]);
            pass.setFilterFieldName ("y");
            pass.setFilterLimits (max, min);
            pass.filter(*objects[i]);

            // Find line
            std::vector<std::pair<geometry_msgs::Point32, geometry_msgs::Point32> > lines;
            for (size_t p = 0; p < objects[i]->size(); p++) {
                geometry_msgs::Point32 newPoint;
                newPoint.x = objects[i]->points[p].z;
                newPoint.y = objects[i]->points[p].x;

                bool added = false;

                for (size_t j = 0; j < lines.size(); j++) {
                    if (lines[j].first.x == lines[j].second.x && lines[j].first.y == lines[j].second.y) {
                        // This only has a start
                        lines[j] = std::make_pair<geometry_msgs::Point32, geometry_msgs::Point32>(lines[j].first, newPoint);
                        added = true;
                        break;
                    } else {
                        // TODO: Check if the angel is too high!

                    }
                }

                if (!added) {
                    lines.push_back(std::make_pair<geometry_msgs::Point32, geometry_msgs::Point32>(newPoint, newPoint));
                }
            }

            for (size_t j = 0; j < lines.size(); j++) {
                if (lines[j].first.x == lines[j].second.x && lines[j].first.y == lines[j].second.y) {
                    continue;
                }

                geometry_msgs::PolygonStamped poly;

                poly.polygon.points.push_back(lines[j].first);
                poly.polygon.points.push_back(lines[j].second);

                obstacle_pub.publish(poly);
            }
        }
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
