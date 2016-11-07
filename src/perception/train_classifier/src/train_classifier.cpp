#include <ros/ros.h>

#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>

#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>


#include <string>

#include <pcl/features/cvfh.h>
#include <pcl/features/our_cvfh.h>

#include <pcl/features/normal_3d.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>

#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <fstream>

#include <ros/package.h>

#include <pcl/apps/render_views_tesselated_sphere.h>
#include <pcl/io/vtk_lib_io.h>
#include <vtkPolyDataMapper.h>

typedef std::pair<std::string, std::vector<float> > vfh_model;

std::vector<std::string> objects;

std::string views_dir;
std::string view_extension = ".pcd";

std::string vfhs_dir;
std::string vfhs_extension = ".pcd";

std::string training_dir;
std::string kdtree_idx_file_name = "kdtree.idx";
std::string training_data_h5_file_name = "training_data.h5";
std::string training_data_list_file_name = "training_data.list";

std::string whatTypeOfData;

void initParams(ros::NodeHandle n) {
    n.getParam("/whatTypeOfData", whatTypeOfData);

    views_dir = ros::package::getPath("train_classifier") + "/Data/Views/" + whatTypeOfData;
    vfhs_dir = ros::package::getPath("train_classifier") + "/Data/VFHS/" + whatTypeOfData;
    training_dir = ros::package::getPath("classifier") + "/Data/Training/" + whatTypeOfData;
}

void loadVFHModels(std::vector<vfh_model> & models) {
    // TODO: Use the one in cvfh instead!
    for (size_t i = 0; i < objects.size(); i++) {
        size_t j = 0;
        while (true) {
            std::string dir = vfhs_dir + "/" + objects[i] + "/" + boost::lexical_cast<std::string>(j) + vfhs_extension;

            // Load the file as PCD
            pcl::PCLPointCloud2 cloud;
            int version;
            Eigen::Vector4f origin;
            Eigen::Quaternionf orientation;
            pcl::PCDReader r;
            int type;
            unsigned int idx;
            if (0 != r.readHeader(dir, cloud)) {
                // Have gone through all files in dir.
                break;
            }

            // Treat the VFH signature as a single Point Cloud
            pcl::PointCloud<pcl::VFHSignature308> point;
            pcl::io::loadPCDFile (dir, point);

            vfh_model vfh;
            vfh.second.resize(308);

            std::vector<pcl::PCLPointField> fields;
            pcl::getFieldIndex(point, "vhf", fields);

            for (size_t k = 0; k < fields[pcl::getFieldIndex(cloud, "vfh")].count; k++) {
                vfh.second[k] = point.points[0].histogram[k];
            }
            vfh.first = objects[i]; // Maybe something else so we know the pose?

            models.push_back(vfh);

            j++;
        }
    }
}

void loadViews(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & views, std::string object) {
    boost::filesystem::path p(views_dir + "/" + object);

    for (boost::filesystem::directory_iterator i = boost::filesystem::directory_iterator(p); i != boost::filesystem::directory_iterator(); i++) {
        if (!boost::filesystem::is_directory(i->path())) {
            std::string fileName = i->path().filename().string();

            if (fileName.find(view_extension) != std::string::npos) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

                pcl::io::loadPCDFile<pcl::PointXYZ>(views_dir + "/" + object + "/" + fileName, *cloud);

                views.push_back(cloud);
            }
        }
    }
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
    pcl::OURCVFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> cvfh;
    cvfh.setInputCloud(cloud);

    // Get normals for cloud
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    computeCloudNormals(cloud, cloud_normals);

    cvfh.setInputNormals(cloud_normals);

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

    cvfh.setSearchMethod(tree);

    cvfh.setCurvatureThreshold(0.025f);
    cvfh.setClusterTolerance(0.015f);   // 1.5 cm, three times the leaf size
    cvfh.setEPSAngleThreshold(0.13f);

    //cvfh.setKSearch(100);   // ???

    //cvfh.setEPSAngleThreshold(angle);
    //cvfh.setCurvatureThreshold(max_curv);
    cvfh.setNormalizeBins(true);

    cvfh.compute(*vfhs);

    /*
    std::cout << "output points.size (): " << vfhs->points.size () << std::endl; // This outputs 1 - should be 397!

    // Display and retrieve the shape context descriptor vector for the 0th point.
    pcl::VFHSignature308 descriptor = vfhs->points[0];
    std::cout << descriptor << std::endl;
    */
}

void train() {
    for (size_t i = 0; i < objects.size(); i++) {
        std::cout << "Processing: " << objects[i] << std::endl;

        // Create directory
        boost::filesystem::path dir(vfhs_dir + "/" + objects[i]);
        boost::filesystem::remove_all(dir); // Remove first so we can add new!
        boost::filesystem::create_directories(dir);

        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> views;

        loadViews(views, objects[i]);

        // For each view calculate CVFH
        for (size_t j = 0; j < views.size(); j++) {
            pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308>);
            // TODO: Maybe do voxel grid first?!
            /*
            pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
            approximate_voxel_filter.setLeafSize (0.005, 0.005, 0.005);
            approximate_voxel_filter.setInputCloud (views[j]);
            approximate_voxel_filter.filter (*views[j]);
            */
            pcl::VoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
            approximate_voxel_filter.setLeafSize (0.005, 0.005, 0.005); //(0.25, 0.25, 0.25);
            approximate_voxel_filter.setInputCloud (views[j]);
            approximate_voxel_filter.filter (*views[j]);

            computeVFHSignature(vfhs, views[j]);

            // Save VFH signature to file
            // TODO: Maybe save pose too?!
            // Save and use the views name instead?!
            pcl::io::savePCDFileASCII(vfhs_dir + "/" + objects[i] + "/" + boost::lexical_cast<std::string>(j) + vfhs_extension, *vfhs);
        }
    }

    // Load all histograms of all models
    std::vector<vfh_model> models;
    loadVFHModels(models);

    // Convert data into FLANN format
    flann::Matrix<float> data (new float[models.size() * models[0].second.size()], models.size(), models[0].second.size());

    for (size_t i = 0; i < data.rows; i++) {
        for (size_t j = 0; j < data.cols; j++) {
            data[i][j] = models[i].second[j];
        }
    }

    // Save data to disk (list of models)
    boost::filesystem::path dir(training_dir);
    boost::filesystem::remove_all(dir); // Remove first so we can add new!
    boost::filesystem::create_directories(dir);
    flann::save_to_file (data, training_dir + "/" + training_data_h5_file_name, "training_data");

    std::ofstream fs;
    fs.open ((training_dir + "/" + training_data_list_file_name).c_str());
    for (size_t i = 0; i < models.size (); i++) {
      fs << models[i].first << "\n";
    }
    fs.close ();

    // Build the tree index and save it to disk
    flann::Index<flann::ChiSquareDistance<float> > index (data, flann::LinearIndexParams());
    //flann::Index<flann::ChiSquareDistance<float> > index (data, flann::KDTreeIndexParams (4));    // Faster but only approximative

    index.buildIndex();
    index.save(training_dir + "/" + kdtree_idx_file_name);
}

void getObjects() {
    boost::filesystem::path p(views_dir);

    for (boost::filesystem::directory_iterator i = boost::filesystem::directory_iterator(p); i != boost::filesystem::directory_iterator(); i++) {
        if (boost::filesystem::is_directory(i->path())) {
            std::string dirName = i->path().filename().string();

            objects.push_back(dirName);
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "train_classifier");
    ros::NodeHandle nh;

    initParams(nh);

    getObjects();

    train();
}
