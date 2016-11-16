// ROS
#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/Image.h>
#include <pcl_ros/point_cloud.h>
#include "std_msgs/String.h"
#include <ros/package.h>

#include "vision/Object.h"

// Sync two topics
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

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

// PCL to remove ground
#include <pcl/filters/passthrough.h>

// PCL Features
#include <pcl/features/normal_3d.h>
//#include <pcl/features/cvfh.h>
//#include <pcl/features/our_cvfh.h>
#include <pcl/features/vfh.h>

// PCL Filters
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

// PCL Common
#include <pcl/common/transforms.h>

// Own
#include "point_cloud_helper.h"
#include "hsv_color.h"

#include "geometry_msgs/PointStamped.h"

// FLANN
#include <flann/flann.h>
#include <flann/io/hdf5.h>

// OpenCV
#include "opencv2/imgproc/imgproc.hpp"
#include "sensor_msgs/PointCloud2.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


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

    training_dir = ros::package::getPath("vision") + "/Data/Training/" + whatTypeOfData;
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
    double lowCertainty = 300;
    double highCertainty = 500;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);

    cloud_xyz->points.resize(cloud_in->points.size());
    for (size_t i = 0; i < cloud_in->points.size(); i++) {
        cloud_xyz->points[i].x = cloud_in->points[i].x;
        cloud_xyz->points[i].y = cloud_in->points[i].y;
        cloud_xyz->points[i].z = cloud_in->points[i].z;
    }

    std::vector<std::pair<std::string, float> > candidates;

    classify(cloud_xyz, candidates);

    if (candidates.size() > 0 && candidates[0].second > highCertainty) {
        //std::cout << "Uncertainty is: " << candidates[0].second << std::endl;
        return "noop";
    }

    if (color == "red") {
        // Red Cube
        // Red Hollow Cube
        // Red Ball

        for (size_t i = 0; i < candidates.size(); i++) {
            if (candidates[i].second > lowCertainty) {
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
            if (candidates[i].second > lowCertainty) {
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
            if (candidates[i].second > lowCertainty) {
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
            if (candidates[i].second > lowCertainty) {
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
            if (candidates[i].second > lowCertainty) {
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

    return "noop";

    // TODO
    //std::cout << classify(cloud_xyz) << std::endl;
    //return classify(cloud_xyz);
}

// GREEN
int iLowH = 33;      // 0
int iHighH = 66;    // 179

int iLowS = 90;    // 0
int iHighS = 255;   // 255

int iLowV = 77;      // 0
int iHighV = 255;   // 255

cv::Mat imgThresholded;
double x = -1;
double y = -1;

#include <opencv2/opencv.hpp>

void getColorPosition(const sensor_msgs::ImageConstPtr & image, std::map<std::string, std::vector<cv::Point> > & colorPosition) {
    cv::Mat imgOriginal = cv_bridge::toCvShare(image, "bgr8")->image;

    cv::Mat imgHSV;

    cv::cvtColor(imgOriginal, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    for (size_t i = 0; i < colorNames.size(); i++) {
        // OpenCV uses  H: 0 - 180, S: 0 - 255, V: 0 - 255
        double hueMin = colors[i].hueMin / 2;
        double hueMax = colors[i].hueMax / 2;
        double satMin = colors[i].saturationMin * 255;
        double satMax = colors[i].saturationMax * 255;
        double valMin = colors[i].valueMin * 255;
        double valMax = colors[i].valueMax * 255;

        cv::Mat oneColored;

        cv::inRange(imgHSV, cv::Scalar(hueMin, satMin, valMin), cv::Scalar(hueMax, satMax, valMax), oneColored); //Threshold the image


        //std::vector<cv::Point> locations;   // output, locations of non-zero pixels
        cv::findNonZero(oneColored, colorPosition[colorNames[i]]);

        // Remove outliers!
        /*
        std::cout << locations.size() << std::endl;

        std::vector<uchar> mask;
        cv::findHomography(oneColored, imgHSV); //, mask);

        cv::imshow("One Color", oneColored);
        cv::waitKey(0);
        */
    }

}

std::string getMostLikelyColor(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_cluster, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_original, std::map<std::string, std::vector<cv::Point> > & colorPosition) {
    // Take points randomly and check closest color?!
    int k = 5; // How many points to sample
    double maxDistSquaredFromColor = 500; //1000000;
    maxDistSquaredFromColor *= maxDistSquaredFromColor;

    std::map<std::string, int> colorsOccurrence;

    int max = cloud_cluster->points.size();

    for (int i = 0; i < k; i++) {
        int random = rand() % max;

        pcl::PointXYZ point = cloud_cluster->points[random];

        // Find the closest point in the original cloud
        int bestIndex = -1;
        double minDist;
        for (size_t j = 0; j < cloud_original->points.size(); j++) {
            double curDist = pcl::squaredEuclideanDistance(point, cloud_original->points[j]);
            if (bestIndex == -1 || curDist < minDist) {
                minDist = curDist;
                bestIndex = j;
            }
        }

        // Get the corresponding image coordinates
        int heightIndex = bestIndex / cloud_original->width;
        int widthIndex = bestIndex % cloud_original->width;

        // Find the closest color
        std::string color;
        double minDistance = -1;
        for (std::map<std::string, std::vector<cv::Point> >::iterator it = colorPosition.begin(); it != colorPosition.end(); it++) {
            for (size_t p = 0; p < it->second.size(); p++) {
                double curDistance = ((heightIndex - it->second[p].y) * (heightIndex - it->second[p].y)) + ((widthIndex - it->second[p].x) * (widthIndex - it->second[p].x));
                if (curDistance < maxDistSquaredFromColor && (minDistance == -1 || curDistance < minDistance)) {
                    color = it->first;
                    minDistance = curDistance;
                } else if (curDistance > maxDistSquaredFromColor) {
                    //std::cout << "Distance is: " << curDistance << std::endl;
                }
            }
        }

        if (minDistance == -1) {
            continue; // Too far away from a color!
        }

        if (colorsOccurrence.find(color) == colorsOccurrence.end()) {
            // Not in the map yet
            colorsOccurrence[color] = 1;
        } else {
            colorsOccurrence[color]++;
        }
    }

    // We will guess it is the color that has most entries in the array
    std::string color = "noop";
    int times = -1;
    for (std::map<std::string, int>::iterator it = colorsOccurrence.begin(); it != colorsOccurrence.end(); it++) {
        if (times == -1 || it->second > times) {
            color = it->first;
            times = it->second;
        }
    }

    return color;
}

ros::Publisher pub;

void callback(const sensor_msgs::ImageConstPtr & rgb, const sensor_msgs::PointCloud2ConstPtr & cloud_in) {
    // IMAGE
    // Check if we can see an object with rgb
    std::map<std::string, std::vector<cv::Point> > colorPosition;
    getColorPosition(rgb, colorPosition);

    /*
    std::cout << rgb->header.stamp - cloud_in->header.stamp << std::endl;
    */

    // POINT CLOUD
    // Convert to PCL
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*cloud_in, pcl_pc);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_original (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromPCLPointCloud2(pcl_pc, *cloud_original);

    // Filtering input scan to increase speed of registration.
    pcl::VoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize (0.005, 0.005, 0.005);
    approximate_voxel_filter.setInputCloud (cloud_original);
    approximate_voxel_filter.filter (*cloud);

    // Remove NaNs
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

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

    Eigen::Quaternionf rotation(w, x, y, z);

    pcl::transformPointCloud(*cloud, *cloud_transformed, translation, rotation);

    // Remove ground
    pcl::PassThrough<pcl::PointXYZ> pass(true); // True because I want the removed indices?!
    pass.setInputCloud (cloud_transformed);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (0.0, 1000.0);
    pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_transformed);

    pcl::PointIndices::Ptr ground (new pcl::PointIndices);

    pass.getRemovedIndices(*ground);

    //std::cout << "Ground: " << ground->indices.size() << std::endl;

    // Remove ground from non-transformed point cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (ground);
    extract.setNegative (true);
    extract.filter (*cloud);

    // Remove outliers
    PointCloudHelper::removeOutliers(cloud, cloud);

    // Seperate (Segmentation)
    std::vector<pcl::PointIndices> cluster_indices = PointCloudHelper::segmentation(cloud);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
            cloud_cluster->points.push_back(cloud->points[*pit]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        cloud_cluster->header = cloud->header; // Will fuck up RVIZ if not here!

        // Get most likely color of cluster
        std::string color = getMostLikelyColor(cloud_cluster, cloud_original, colorPosition);

        if (color == "noop") {
            // TODO
            // Should see if it is another obstacle
            continue; // Can not classify this!
        }

        // Classify
        std::string objectType = classify(cloud_cluster, color);
        if (objectType == "noop") {
            // TODO
            continue;
        }
        //std::cout << color << " " << objectType << std::endl;

        // TODO
        // Transform before calculating the optimal pickup point
        pcl::transformPointCloud(*cloud_cluster, *cloud_cluster, translation, rotation);
        vision::Object object = PointCloudHelper::getOptimalPickupPoint(cloud_cluster);

        object.y = -object.y;
        std::cout << color << " " << objectType << " at: " << "X: " << object.x << ", Y: " << object.y << ", Z: " << object.z << std::endl;

        object.color = color;
        object.type = 1;

        // Output

        geometry_msgs::PointStamped point;
        point.point.x = object.x;
        point.point.y = object.y;
        point.point.z = object.z;
        point.header.frame_id = "wheel_center";
        point.header.stamp = ros::Time();

        object_pub.publish(object);

        // Output
        sensor_msgs::PointCloud2 output;

        pcl::toROSMsg(*cloud_cluster, output);

        pub.publish(output);

        /*
        sensor_msgs::Image test;
        pcl::toROSMsg(output, test);

        cv::Mat imgOriginal = cv_bridge::toCvCopy(test, "mono8")->image;

        cv::imshow("Test", imgOriginal);
        cv::waitKey(0);
        */
        break;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    initParams(nh);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> depth_sub(nh, "/camera/depth/points", 1);

    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> > sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2>(10), rgb_sub, depth_sub);

    sync.registerCallback(boost::bind(&callback, _1, _2));

    pub = nh.advertise<sensor_msgs::PointCloud2>("calibrate", 1);

    /*
    ros::Subscriber point_sub = nh.subscribe("camera/depth_registered/points", 1, pointCloudCallback);
    */
    object_pub = nh.advertise<vision::Object> ("objectPos_wheelcenter2", 1);
    point_pub = nh.advertise<geometry_msgs::PointStamped> ("objectPos_wheelcenter", 1);

    // Test if this work!
    /*
    ros::Rate loop_rate(10.0);

    while (ros::ok()) {
        ros::spinOnce();

        loop_rate.sleep();
    }
    */
    ros::spin();
}

void callback2(const sensor_msgs::ImageConstPtr & rgb, const sensor_msgs::PointCloud2ConstPtr & cloud_in) {
    cv::Mat imgOriginal = cv_bridge::toCvShare(rgb, "bgr8")->image;

    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*cloud_in, pcl_pc);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::fromPCLPointCloud2(pcl_pc, *cloud);

    size_t k = 0; // cloud->points.size() - 0;
    for (size_t i = 0; i < rgb->height; i++) {
        for (size_t j = 0; j < rgb->width; j++, k++) {
            k = k % cloud->points.size();
            if (isnan (cloud->points[k].x) || isnan (cloud->points[k].y) || isnan (cloud->points[k].z)) {
                    continue;
            }
            cv::Vec3b bgrPixel = imgOriginal.at<cv::Vec3b>(i, j);
            //int32_t rgb = (g << 16) | (g << 8) | g;
            //cloud->points[k]
            //cloud->points[k].rgb = *(float*)(&rgb);
            cloud->points[k].r = bgrPixel.val[2];
            cloud->points[k].g = bgrPixel.val[1];
            cloud->points[k].b = bgrPixel.val[0];
        }
    }

    // Output
    sensor_msgs::PointCloud2 output;

    pcl::toROSMsg(*cloud, output);

    pub.publish(output);
}
