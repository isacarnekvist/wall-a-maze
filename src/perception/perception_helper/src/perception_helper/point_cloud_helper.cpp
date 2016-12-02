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

// PCL Segmentation
#include <pcl/segmentation/extract_clusters.h>

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

    std::vector<pcl::PointIndices> segmentation(pcl_rgb::Ptr & cloud_in, double clusterTolerance, double minClusterSize, double maxClusterSize) {
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

        return cluster_indices;
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

    std::vector<pcl_rgb::Ptr> getObjects(pcl_rgb::Ptr & cloud_in, hsvColor color, int outlierMaxNeighbours, double outlierStddev, double clusterTolerance, double minClusterSize, double maxClusterSize) {
        std::vector<pcl_rgb::Ptr> objects;

        pcl_rgb::Ptr filtered_cloud (new pcl_rgb);

        // Filter on color
        //std::cout << "Filter on color" << std::endl;
        PointCloudHelper::HSVFilter(cloud_in, filtered_cloud, color);

        // Remove NaNs
        //std::cout << "Remove NaNs from cloud" << std::endl;
        std::vector<int> indicesTemp;
        pcl::removeNaNFromPointCloud(*filtered_cloud, *filtered_cloud, indicesTemp);

        // Check if cloud is empty
        //std::cout << "Check if cloud is empty" << std::endl;
        if (filtered_cloud->points.size() == 0) {
            //std::cout << "Cloud empty" << std::endl;
            return objects;
        }

        // Remove outliers
        //std::cout << "Remove outliers" << std::endl;
        pcl::PointIndices::Ptr outliers_indices (new pcl::PointIndices());  // TODO: continue?!
        PointCloudHelper::removeOutliers(filtered_cloud, filtered_cloud, outlierMaxNeighbours, outlierStddev);

        // Check if cloud is empty
        //std::cout << "Check if cloud is empty" << std::endl;
        if (filtered_cloud->points.size() == 0) {
            //std::cout << "Cloud empty" << std::endl;
            return objects;
        }

        // Seperate (Segmatation)
        //std::cout << "Segment cloud into clusters" << std::endl;
        std::vector<pcl::PointIndices> cluster_indices = PointCloudHelper::segmentation(filtered_cloud, clusterTolerance, minClusterSize, maxClusterSize);

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++) {
            pcl_rgb::Ptr cluster_cloud (new pcl_rgb);
            //std::cout << "Create a cluster object" << std::endl;
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
                cluster_cloud->points.push_back(filtered_cloud->points[*pit]);
            }

            cluster_cloud->width = cluster_cloud->points.size();
            cluster_cloud->height = 1;
            cluster_cloud->is_dense = true;
            cluster_cloud->header = filtered_cloud->header;

            objects.push_back(cluster_cloud);
        }

        return objects;
    }
}
