#include "point_cloud_helper.h"

#include <pcl/PointIndices.h>
#include <pcl/point_types_conversion.h>

// PCL Filters
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

// PCL Segmentation
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/common/common.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


namespace PointCloudHelper {
    void HSVFilter(pcl_rgb::Ptr cloud_in, pcl_rgb::Ptr cloud_out, hsvColor color) {

        pcl::PointIndices::Ptr indices (new pcl::PointIndices());

        for (size_t i = 0; i < cloud_in->points.size(); i++) {
            pcl::PointXYZHSV hsv_point;
            pcl::PointXYZRGBtoXYZHSV(cloud_in->points[i], hsv_point);

            if (hsv_point.h >= color.hueMin && hsv_point.h <= color.hueMax && hsv_point.s >= color.saturationMin && hsv_point.s <= color.saturationMax && hsv_point.v >= color.valueMin && hsv_point.v <= color.valueMax) {
                indices->indices.push_back(i);
            }
        }

        pcl::ExtractIndices<pcl::PointXYZRGB> extract;

        extract.setInputCloud(cloud_in);
        extract.setIndices(indices);
        extract.setNegative(false);
        extract.filter(*cloud_out);
    }

    void removeOutliers(pcl_rgb::Ptr cloud_in, pcl_rgb::Ptr cloud_out, int numNeighbours, double stddev) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cloud_in);
        sor.setMeanK(numNeighbours);
        sor.setStddevMulThresh(stddev);
        sor.filter(*cloud_out);
    }

    void removeOutliers(pcl_xyz::Ptr cloud_in, pcl_xyz::Ptr cloud_out, int numNeighbours, double stddev) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud_in);
        sor.setMeanK(numNeighbours);
        sor.setStddevMulThresh(stddev);
        sor.filter(*cloud_out);
    }

    std::vector<pcl::PointIndices> segmentation(pcl_rgb::Ptr cloud_in) {
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
        tree->setInputCloud(cloud_in);
        std::vector<pcl::PointIndices> cluster_indices;

        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance (0.02); // 2cm
        ec.setMinClusterSize (100);
        ec.setMaxClusterSize (25000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_in);
        ec.extract (cluster_indices);

        return cluster_indices;
    }

    std::vector<pcl::PointIndices> segmentation(pcl_xyz::Ptr cloud_in) {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_in);
        std::vector<pcl::PointIndices> cluster_indices;

        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.02); // 2cm
        ec.setMinClusterSize (100);
        ec.setMaxClusterSize (25000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_in);
        ec.extract (cluster_indices);

        return cluster_indices;
    }
}
