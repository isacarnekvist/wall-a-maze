#ifndef POINT_CLOUD_HELPER_H
#define POINT_CLOUD_HELPER_H

// Own
#include "perception_helper/hsv_color.h"

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

namespace PointCloudHelper {

    #define pcl_rgb pcl::PointCloud<pcl::PointXYZRGB>

    void HSVFilter(pcl_rgb::Ptr & cloud_in, pcl_rgb::Ptr & cloud_out, hsvColor color);

    void HSVFilter(pcl_rgb::Ptr & cloud_in, pcl_rgb::Ptr & cloud_out, hsvColor color, pcl::PointIndices::Ptr & indices);

    void removeOutliers(pcl_rgb::Ptr & cloud_in, pcl_rgb::Ptr & cloud_out, int numNeighbours = 20, double stddev = 1.0);

    void removeOutliers(pcl_rgb::Ptr & cloud_in, pcl_rgb::Ptr & cloud_out, pcl::PointIndices::Ptr & indices, int numNeighbours = 20, double stddev = 1.0);

    std::vector<pcl::PointIndices> segmentation(pcl_rgb::Ptr & cloud_in, double clusterTolerance = 0.02, double minClusterSize = 100, double maxClusterSize = 25000);

    void resizePoints(const pcl_rgb::ConstPtr & cloud_in, pcl_rgb::Ptr & cloud_out, double xSize = 0.005, double ySize = 0.005, double zSize = 0.005);

    void resizePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_out, double xSize = 0.005, double ySize = 0.005, double zSize = 0.005);

    std::vector<pcl_rgb::Ptr> getObjects(pcl_rgb::Ptr & cloud_in, hsvColor color, int outlierMaxNeighbours, double outlierStddev, double clusterTolerance, double minClusterSize, double maxClusterSize);

    std::vector<pcl_rgb::Ptr> getObjects(pcl_rgb::Ptr & cloud_in, hsvColor color, pcl::PointIndices::Ptr & indices, int outlierMaxNeighbours, double outlierStddev, double clusterTolerance, double minClusterSize, double maxClusterSize);
}

#endif // POINT_CLOUD_HELPER_H
