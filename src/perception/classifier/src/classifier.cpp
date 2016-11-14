// ROS
#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include "std_msgs/String.h"
#include <ros/package.h>

#include "classifier/Object.h"

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
#include <pcl/features/normal_3d.h>
//#include <pcl/features/cvfh.h>
//#include <pcl/features/our_cvfh.h>
#include <pcl/features/vfh.h>

// PCL Filters
#include <pcl/filters/voxel_grid.h>

// PCL Common
#include <pcl/common/transforms.h>

// Own
#include "point_cloud_helper.h"
#include "hsv_color.h"

#include "geometry_msgs/PointStamped.h"

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
std::vector<std::string> colorNames;

ros::Publisher object_pub;
ros::Publisher point_pub;

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

    n.getParam("/color_names", colorNames);

    for (int i = 0; i < colorNames.size(); i++) {
        std::map<std::string, double> color_hsv;

        n.getParam("/" + colorNames[i] + "_hsv", color_hsv);

        hsvColor color = { color_hsv["h_min"], color_hsv["h_max"], color_hsv["s_min"], color_hsv["s_max"], color_hsv["v_min"], color_hsv["v_max"] };

        colors.push_back(color);
    }

    std::string whatTypeOfData;
    n.getParam("/whatTypeOfData", whatTypeOfData);

    training_dir = ros::package::getPath("classifier") + "/Data/Training/" + whatTypeOfData;
}

/** \brief Load the list of file model names from an ASCII file
  * \param models the resultant list of model name
  * \param filename the input file name
  */
bool loadFileList (std::vector<vfh_model> &models) {
  ifstream fs;
  fs.open ((training_dir + "/" + training_data_list_file_name).c_str());
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

void computeCloudNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, pcl::PointCloud<pcl::Normal>::Ptr & cloud_normals) {
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);


    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.03);

    // Compute the features
    ne.compute (*cloud_normals);

    // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
}

void computeVFHSignature(pcl::PointCloud<pcl::VFHSignature308>::Ptr & vfhs, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud) {
    pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> cvfh;
    cvfh.setInputCloud(cloud);

    // Get normals for cloud
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    computeCloudNormals(cloud, cloud_normals);
    cvfh.setInputNormals(cloud_normals);

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

    cvfh.setSearchMethod(tree);

    //cvfh.setCurvatureThreshold(0.025f);
    //cvfh.setClusterTolerance(0.015f);   // 1.5 cm, three times the leaf size
    //cvfh.setEPSAngleThreshold(0.13f);

    //cvfh.setKSearch(100);   // ???

    //cvfh.setEPSAngleThreshold(angle);
    //cvfh.setCurvatureThreshold(max_curv);
    cvfh.setNormalizeBins(false);

    cvfh.compute(*vfhs);

    /*
    std::cout << "output points.size (): " << vfhs->points.size () << std::endl; // This outputs 1 - should be 397!

    // Display and retrieve the shape context descriptor vector for the 0th point.
    pcl::VFHSignature308 descriptor = vfhs->points[0];
    std::cout << descriptor << std::endl;
    */
}

void nearestKSearch (flann::Index<flann::ChiSquareDistance<float> > &index, const vfh_model &model,
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

    computeVFHSignature(vfhs, cloud);

    //pcl::io::savePCDFileASCII(base_dir + "/Views/Real/1" + data_extension, *vfhs);
    //pcl::io::savePCDFileASCII(data_dir + "/" + "Test" + "/" + "Test" + data_extension, *vfhs);

    generateHistogram(vfh, vfhs);
}

void classify(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, std::vector<std::pair<std::string, float> > & candidates) {


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



    int k = 50;

    std::vector<vfh_model> models;
    flann::Matrix<int> k_indices;
    flann::Matrix<float> k_distances;
    flann::Matrix<float> data;

    loadFileList(models);
    flann::load_from_file(data, training_dir + "/training_data.h5", "training_data");

    vfh_model histogram;
    //loadVFHModel(histogram, "Cube", "5"); // TODO

    generateHistogram(histogram, cloud);



    flann::Index<flann::ChiSquareDistance<float> > index (data, flann::SavedIndexParams(training_dir + "/kdtree.idx"));
    index.buildIndex();
    nearestKSearch(index, histogram, k, k_indices, k_distances);

    for (int i = 0; i < k; i++) {
        //std::cout << i << " - " << models.at(k_indices[0][i]).first << " (" << k_indices[0][i] << ")" << " with a distance of: " << k_distances[0][i] << std::endl;
    }

    for (size_t i = 0; i < k; i++) {
        candidates.push_back(std::make_pair(models.at(k_indices[0][i]).first, k_distances[0][i]));
    }


    //return candidates;
}

std::string classify(pcl_rgb::Ptr cloud_in, std::string color) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);

    cloud_xyz->points.resize(cloud_in->points.size());
    for (size_t i = 0; i < cloud_in->points.size(); i++) {
        cloud_xyz->points[i].x = cloud_in->points[i].x;
        cloud_xyz->points[i].y = cloud_in->points[i].y;
        cloud_xyz->points[i].z = cloud_in->points[i].z;
    }

    std::vector<std::pair<std::string, float> > candidates;

    classify(cloud_xyz, candidates);

    if (color == "red") {
        // Red Cube
        // Red Hollow Cube
        // Red Ball

        for (size_t i = 0; i < candidates.size(); i++) {
            if (candidates[i].second > 300) {
                return "object"; // We are too unsure
            }

            if (candidates[i].first == "Cube") {
                return "Cube";
            } else if (candidates[i].first == "Sphere" || candidates[i].first == "Ball") {
                return "Ball";
            }
        }

    } else if (color == "blue") {
        // Blue Cube
        // Blue Triangle
        for (size_t i = 0; i < candidates.size(); i++) {
            if (candidates[i].second > 300) {
                return "object"; // We are too unsure
            }

            if (candidates[i].first == "Cube") {
                return "Cube";
            } else if (candidates[i].first == "Triangle") {
                return "Triangle";
            }
        }

    } else if (color == "purple") {
        // Purple Cross
        // Purple Star

        for (size_t i = 0; i < candidates.size(); i++) {
            if (candidates[i].second > 300) {
                return "object"; // We are too unsure
            }

            if (candidates[i].first == "Cross") {
                return "Cross";
            } else if (candidates[i].first == "Star") {
                return "Star";
            }
        }

    } else if (color == "yellow") {
        // Yellow Cube
        // Yellow Ball

        for (size_t i = 0; i < candidates.size(); i++) {
            if (candidates[i].second > 300) {
                return "object"; // We are too unsure
            }

            if (candidates[i].first == "Cube") {
                return "Cube";
            } else if (candidates[i].first == "Ball") {
                return "Ball";
            }
        }

    } else if (color == "green") {
        // Green Cube
        // Green Cylinder

        for (size_t i = 0; i < candidates.size(); i++) {
            if (candidates[i].second > 300) {
                return "object"; // We are too unsure
            }

            if (candidates[i].first == "Cube") {
                return "Cube";
            } else if (candidates[i].first == "Cylinder") {
                return "Cylinder";
            }
        }
    } else if (color == "orange") {
        // TODO: Still have to check?!
        /*
        for (size_t i = 0; i < candidates.size(); i++) {
            if (candidates[i].second > 300) {
                return "object"; // We are too unsure
            }

            if (candidates[i].first == "Star") {
                return patric;
            }
        }
        */

        return "patric";
    }

    return "object";

    // TODO
    //std::cout << classify(cloud_xyz) << std::endl;
    //return classify(cloud_xyz);
}

std::vector<std::string> said;

void pointCloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input_cloud) {
    // Filtering input scan to increase speed of registration.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::VoxelGrid<pcl::PointXYZRGB> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize (0.005, 0.005, 0.005);
    approximate_voxel_filter.setInputCloud (input_cloud);
    approximate_voxel_filter.filter (*filtered_cloud);

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

    //pcl::transformPointCloud(*filtered_cloud, *filtered_cloud, translation, rotation);


    // Remove NaNs
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*filtered_cloud, *filtered_cloud, indices);

    // Find the color objects
    for (int i = 0; i < colors.size(); i++) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

        *color_cloud = *filtered_cloud;

        //std::cout << color_cloud->points.size() << ", " << filtered_cloud->points.size() << std::endl;

        // Filter on color
        PointCloudHelper::HSVFilter(color_cloud, color_cloud, colors[i]);

        // Check if cloud is empty
        if (color_cloud->points.size() == 0) {
            continue;
        }

        // Remove outliers
        PointCloudHelper::removeOutliers(color_cloud, color_cloud);

        // Check if cloud is empty
        if (color_cloud->points.size() == 0) {
            continue;
        }

        // Seperate (Segmatation)
        std::vector<pcl::PointIndices> cluster_indices = PointCloudHelper::segmentation(color_cloud);
        //std::cout << cluster_indices.size() << std::endl;

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
                cluster_cloud->points.push_back(color_cloud->points[*pit]);
            }
            cluster_cloud->width = cluster_cloud->points.size();
            cluster_cloud->height = 1;
            cluster_cloud->is_dense = true;
            cluster_cloud->header = color_cloud->header; // Will fuck up RVIZ if not here!


            //std::cout << color_cloud->points.size() << ", " << cluster_cloud->points.size() << std::endl;

            // Classify
            std::string objectType = classify(cluster_cloud, colorNames[i]);
            //std::cout << objectType << std::endl;

            // Filter so only top remains
            // TODO: Do this before classify?!
            pcl::transformPointCloud(*cluster_cloud, *cluster_cloud, translation, rotation);
            classifier::Object object = PointCloudHelper::getOptimalPickupPoint(cluster_cloud);

	    object.y = -object.y;
            std::cout << colorNames[i] << " " << objectType << " at: " << "X: " << object.x << ", Y: " << object.y << ", Z: " << object.z << std::endl;

            object.color = colorNames[i];
            object.type = 1;

            // Output

            geometry_msgs::PointStamped point;
            point.point.x = object.x;
            point.point.y = object.y;
            point.point.z = object.z;
            point.header.frame_id = "wheel_center";
            point.header.stamp = ros::Time();

            object_pub.publish(object);
            //point_pub.publish(point);

            for (size_t i = 0; i < said.size(); i++) {
                if (said[i] == objectType) {
                    return;
                }
            }
            said.push_back(objectType);
        }
    }

    // Find other obstacles
    // Booby trap
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    initParams(nh);

    ros::Subscriber point_sub = nh.subscribe("camera/depth_registered/points", 1, pointCloudCallback);

    object_pub = nh.advertise<classifier::Object> ("objectPos_wheelcenter2", 1);
    point_pub = nh.advertise<geometry_msgs::PointStamped> ("objectPos_wheelcenter", 1);

    // Test if this work!
    ros::Rate loop_rate(10.0);

    while (ros::ok()) {
        ros::spinOnce();

        loop_rate.sleep();
    }
}
