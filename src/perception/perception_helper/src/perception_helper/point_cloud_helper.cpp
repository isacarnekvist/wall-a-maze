// Include the ROS C++ APIs
#include <ros/ros.h>

// Own
#include "perception_helper/point_cloud_helper.h"
#include "perception_helper/hsv_color.h"

// PCL
#include <pcl/point_types_conversion.h>

// PCL Filters
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

// PCL Segmentation
#include <pcl/segmentation/extract_clusters.h>

// PCL Common
#include <pcl/common/transforms.h>

#include <pcl/io/vtk_lib_io.h>

namespace PointCloudHelper {
    void HSVFilter(pcl_rgb::Ptr & cloud_in, pcl_rgb::Ptr & cloud_out, hsvColor color) {
        pcl::PointIndices::Ptr indices (new pcl::PointIndices());

        for (size_t i = 0; i < cloud_in->points.size(); i++) {
            pcl::PointXYZHSV hsv_point;
            pcl::PointXYZRGBtoXYZHSV(cloud_in->points[i], hsv_point);

            if (hsv_point.h >= color.hueMin && hsv_point.h <= color.hueMax && hsv_point.s >= color.saturationMin && hsv_point.s <= color.saturationMax && hsv_point.v >= color.valueMin && hsv_point.v <= color.valueMax) {
                indices->indices.push_back(i);
            }
        }

        pcl::ExtractIndices<pcl::PointXYZRGB> extract;

        // Cloud out will only contain the colored points
        extract.setInputCloud(cloud_in);
        extract.setIndices(indices);
        extract.setNegative(false);
        extract.filter(*cloud_out);

        // Remove the colored points from cloud in
        extract.setInputCloud(cloud_in);
        extract.setIndices(indices);
        extract.setNegative(true);
        extract.filter(*cloud_in);
    }

    void removeOutliers(pcl_rgb::Ptr & cloud_in, pcl_rgb::Ptr & cloud_out, int numNeighbours, double stddev) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cloud_in);
        sor.setMeanK(numNeighbours);
        sor.setStddevMulThresh(stddev);
        sor.filter(*cloud_out);
    }

    void removeOutliers(pcl_rgb::Ptr & cloud_in, pcl_rgb::Ptr & cloud_out, pcl::PointIndices::Ptr & indices, int numNeighbours, double stddev) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cloud_in);
        sor.setMeanK(numNeighbours);
        sor.setStddevMulThresh(stddev);
        sor.filter(*cloud_out);

        sor.getRemovedIndices(*indices);  // Does this work?!
    }

    void resizePoints(const pcl_rgb::ConstPtr & cloud_in, pcl_rgb::Ptr & cloud_out, double xSize, double ySize, double zSize) {
        pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
        voxel_filter.setLeafSize (xSize, ySize, zSize); //(0.25, 0.25, 0.25);
        voxel_filter.setInputCloud (cloud_in);
        voxel_filter.filter (*cloud_out);
    }

    void resizePoints(pcl_rgb::Ptr & cloud_in, pcl_rgb::Ptr & cloud_out, double xSize, double ySize, double zSize) {
        pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
        voxel_filter.setLeafSize (xSize, ySize, zSize); //(0.25, 0.25, 0.25);
        voxel_filter.setInputCloud (cloud_in);
        voxel_filter.filter (*cloud_out);
    }

    void preProcess(pcl_rgb::Ptr & cloud_in, const Eigen::Vector3f translation, Eigen::Quaternionf rotation, std::vector<float> pointSize) {
        // Remove NaNs
        removeNaNs(cloud_in, cloud_in);

        // Remove ground
        removeGround(cloud_in, cloud_in, translation, rotation);

        // Resize
        if (pointSize[0] != 0 || pointSize[1] != 0 || pointSize[2] != 0) {
            resizePoints(cloud_in, cloud_in, pointSize[0], pointSize[1], pointSize[2]);
        }

        // Remove outliers
        // TODO: Needs to be done?!
    }

    void preProcess(pcl_rgb::Ptr & cloud_in, const Eigen::Vector3f translation, Eigen::Quaternionf rotation, std::vector<float> pointSize, hsvColor wallColor) {
        // Remove NaNs
        removeNaNs(cloud_in, cloud_in);

        // Remove ground
        removeGround(cloud_in, cloud_in, translation, rotation);

        // Remove walls
        pcl_rgb::Ptr cloud_temp (new pcl_rgb);
        //HSVFilter(cloud_in, cloud_temp, wallColor);

        // Resize
        if (pointSize[0] != 0 || pointSize[1] != 0 || pointSize[2] != 0) {
            resizePoints(cloud_in, cloud_in, pointSize[0], pointSize[1], pointSize[2]);
        }

        // Remove outliers
        // TODO: Needs to be done?!
    }

    std::vector<pcl_rgb::Ptr> getObjects(pcl_rgb::Ptr & cloud_in, hsvColor color, int outlierMaxNeighbours, double outlierStddev, double clusterTolerance, double minClusterSize, double maxClusterSize) {
        std::vector<pcl_rgb::Ptr> objects;

        pcl_rgb::Ptr filtered_cloud (new pcl_rgb);

        // Remove this color from cloud_in
        //std::cout << "Filter on color" << std::endl;
        HSVFilter(cloud_in, filtered_cloud, color);

        // Remove NaNs
        //std::vector<int> indicesTemp;
        //pcl::removeNaNFromPointCloud(*filtered_cloud, *filtered_cloud, indicesTemp);

        // Check if cloud is empty
        //std::cout << "Check if cloud is empty" << std::endl;
        if (filtered_cloud->points.size() == 0) {
            //std::cout << "Cloud empty" << std::endl;
            return objects;
        }

        // Remove outliers
        //std::cout << "Remove outliers" << std::endl;
        pcl::PointIndices::Ptr outliers_indices (new pcl::PointIndices());  // TODO: continue?!
        removeOutliers(filtered_cloud, filtered_cloud, outlierMaxNeighbours, outlierStddev);

        // Check if cloud is empty
        //std::cout << "Check if cloud is empty" << std::endl;
        if (filtered_cloud->points.size() == 0) {
            //std::cout << "Cloud empty" << std::endl;
            return objects;
        }

        // Seperate (Segmatation)
        //std::cout << "Segment cloud into clusters" << std::endl;
        return segmentation(filtered_cloud, clusterTolerance, minClusterSize, maxClusterSize);
    }

    void removeNaNs(pcl_rgb::Ptr & cloud_in, pcl_rgb::Ptr & cloud_out) {
        std::vector<int> indicesTemp;
        pcl::removeNaNFromPointCloud(*cloud_in, *cloud_out, indicesTemp);
    }

    void removeGround(pcl_rgb::Ptr & cloud_in, pcl_rgb::Ptr & cloud_out, const Eigen::Vector3f & translation, const Eigen::Quaternionf & rotation) {
        pcl_rgb::Ptr cloud_transformed (new pcl_rgb);

        pcl::transformPointCloud(*cloud_in, *cloud_transformed, translation, rotation);

        pcl::PassThrough<pcl::PointXYZRGB> pass(true); // True because I want the removed indices?!
        pass.setInputCloud (cloud_transformed);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (-0.01, 1000.0);
        pass.setFilterLimitsNegative (true);
        pass.filter (*cloud_transformed);

        pcl::PointIndices::Ptr ground (new pcl::PointIndices);

        pass.getRemovedIndices(*ground);

        // Remove ground from non-transformed point cloud
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud (cloud_in);
        extract.setIndices (ground);
        extract.setNegative (true);
        extract.filter (*cloud_out);
    }

    pcl::PointXYZ getOptimalPickupPoint(pcl_rgb::Ptr & cloud_in, std::string objectType) {
        pcl::PointXYZ optimalPoint;

        pcl::PointXYZRGB min_p, max_p;

        pcl::getMinMax3D(*cloud_in, min_p, max_p);

        optimalPoint.x = min_p.z;
        optimalPoint.z = -max_p.y;

        // What if you cannot see the whole object?!
        optimalPoint.y = (min_p.x + ((max_p.x - min_p.x) / 2.0));

        if (objectType == "cube") {
            // Todo, what if the object is too close?!
            optimalPoint.x = min_p.z + 0.02;    // 2 cm from closest point

            optimalPoint.z = -max_p.y + 0.02;   // Max in height
        } else if (objectType == "hollow cube") {
            // Todo, what if the object is too close?!
            optimalPoint.x = min_p.z + 0.02;    // 2 cm from closest point

            optimalPoint.z = -max_p.y;   // It is hollow
        } else if (objectType == "ball") {
            // Todo, what if the object is too close?!
            optimalPoint.x = min_p.z + 0.02;    // 2 cm from closest point

            optimalPoint.z = -max_p.y + 0.025;   // Max in height
        } else if (objectType == "triangle") {
            // Todo, what if the object is too close?!
            optimalPoint.x = min_p.z + 0.02;    // 2 cm from closest point

            optimalPoint.z = -max_p.y;   // Max in height
        } else if (objectType == "star") {
            // Todo, what if the object is too close?!
            optimalPoint.x = min_p.z + 0.02;    // 2 cm from closest point

            optimalPoint.z = -max_p.y;   // Max in height
        } else if (objectType == "cross") {
            // Todo, what if the object is too close?!
            optimalPoint.x = min_p.z + 0.02;    // 2 cm from closest point

            optimalPoint.z = -max_p.y;   // Max in height
        } else if (objectType == "cylinder") {
            // Todo, what if the object is too close?!
            optimalPoint.x = min_p.z + 0.02;    // 2 cm from closest point

            optimalPoint.z = -max_p.y;   // Max in height
        }

        return optimalPoint;
    }

    std::vector<pcl_rgb::Ptr> segmentation(pcl_rgb::Ptr & cloud_in, double clusterTolerance, double minClusterSize, double maxClusterSize) {
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
        tree->setInputCloud(cloud_in);
        std::vector<pcl::PointIndices> cluster_indices;

        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance (clusterTolerance); // 2cm
        ec.setMinClusterSize (minClusterSize);
        ec.setMaxClusterSize (maxClusterSize);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_in);
        ec.extract (cluster_indices);

        std::vector<pcl_rgb::Ptr> objects;

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++) {
            pcl_rgb::Ptr cluster_cloud (new pcl_rgb);
            //std::cout << "Create a cluster object" << std::endl;
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
                cluster_cloud->points.push_back(cloud_in->points[*pit]);
            }

            cluster_cloud->width = cluster_cloud->points.size();
            cluster_cloud->height = 1;
            cluster_cloud->is_dense = true;
            cluster_cloud->header = cloud_in->header;

            objects.push_back(cluster_cloud);
        }

        return objects;
    }

    std::string getDominateColor(pcl_rgb::Ptr & cloud_in, std::vector<hsvColor> & colors, std::vector<std::string> & colorNames) {
        pcl_rgb::Ptr cloud (new pcl_rgb);
        *cloud = *cloud_in;

        std::string dominate = "";
        int points = 10;    // Min limit

        for (size_t i = 0; i < colors.size(); i++) {
            pcl_rgb::Ptr temp (new pcl_rgb);
            HSVFilter(cloud, temp, colors[i]);

            if (temp->size() > points) {
                points = temp->size();
                dominate = colorNames[i];
            }
        }

        return dominate;
    }
}
