#ifndef VFH_HELPER_H
#define VFH_HELPER_H

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace VFHHelper {

    #define pcl_rgb     pcl::PointCloud<pcl::PointXYZ>
    #define PI          3.14159265358979323846  /* pi */

    void computeCloudNormals(pcl_rgb::Ptr & cloud_in, pcl::PointCloud<pcl::Normal>::Ptr & cloud_normals, double radiusSearch = 0.03);

    void computeVFHSignature(pcl::PointCloud<pcl::VFHSignature308>::Ptr & vfhs, pcl_rgb::Ptr & cloud, double radiusSearch = 0.03, bool normalizeBins = false, double EPSAngle = 5.0 / 180.0 + PI, double maxCurv = 1.0);
}

#endif // VFH_HELPER_H
