// ROS
#include <ros/ros.h>

// Own
#include "perception_helper/vfh_helper.h"

// PCL Features
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/vfh.h>



namespace VFHHelper {
    void computeCloudNormals(pcl_rgb::Ptr & cloud_in, pcl::PointCloud<pcl::Normal>::Ptr & cloud_normals, double radiusSearch) {
        // Create the normal estimation class, and pass the input dataset to it
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud (cloud_in);

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        ne.setSearchMethod (tree);

        // Use all neighbors in a sphere of radius 3cm
        ne.setRadiusSearch (radiusSearch);

        ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

        // Compute the features
        ne.compute (*cloud_normals);

        // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
    }

    void computeVFHSignature(pcl::PointCloud<pcl::VFHSignature308>::Ptr & vfhs, pcl_rgb::Ptr & cloud, double radiusSearch, bool normalizeBins, double EPSAngle, double maxCurv) {
        pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> cvfh;
        cvfh.setInputCloud(cloud);

        // Get normals for cloud
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        computeCloudNormals(cloud, cloud_normals, radiusSearch);

        cvfh.setInputNormals(cloud_normals);

        // Create an empty kdtree representation, and pass it to the FPFH estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

        cvfh.setSearchMethod(tree);

        //cvfh.setCurvatureThreshold(0.025f);
        //cvfh.setClusterTolerance(0.015f);   // 1.5 cm, three times the leaf size
        //cvfh.setEPSAngleThreshold(0.13f);

        //cvfh.setKSearch(100);   // ???

        //cvfh.setEPSAngleThreshold(EPSAngle);
        //cvfh.setCurvatureThreshold(maxCurv);
        cvfh.setNormalizeBins(normalizeBins);
        cvfh.setNormalizeDistance(false);

        cvfh.compute(*vfhs);
    }
}
