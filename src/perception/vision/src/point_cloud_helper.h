#ifndef POINT_CLOUD_HELPER_H
#define POINT_CLOUD_HELPER_H

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

#include "vision/Object.h"

#include "hsv_color.h"

namespace PointCloudHelper {

    #define pcl_rgb pcl::PointCloud<pcl::PointXYZ>

    void HSVFilter(pcl_rgb::Ptr cloud_in, pcl_rgb::Ptr cloud_out, hsvColor color);

    vision::Object getOptimalPickupPoint(pcl_rgb::Ptr cloud_in);

    void removeOutliers(pcl_rgb::Ptr cloud_in, pcl_rgb::Ptr cloud_out, int numNeighbours = 20, double stddev = 1.0);

    std::vector<pcl::PointIndices> segmentation(pcl_rgb::Ptr cloud_in);
}

#endif // POINT_CLOUD_HELPER_H
