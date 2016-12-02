// ROS
#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"

// Own
#include <perception_helper/point_cloud_helper.h>
#include <perception_helper/vfh_helper.h>

// PCL
#include <pcl_ros/point_cloud.h>

#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>


#include <string>


#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <fstream>

#include <ros/package.h>

#include <pcl/apps/render_views_tesselated_sphere.h>
#include <pcl/io/vtk_lib_io.h>
#include <vtkPolyDataMapper.h>

typedef std::pair<std::string, std::vector<float> > vfh_model;

std::map<std::string, std::vector<std::string> > objects;

std::string views_dir;
std::string view_extension = ".pcd";

std::string vfhs_dir;
std::string vfhs_extension = ".pcd";

std::string training_dir;
std::string kdtree_idx_file_name = "kdtree.idx";
std::string training_data_h5_file_name = "training_data.h5";
std::string training_data_list_file_name = "training_data.list";

std::string whatTypeOfData;

double normalRadiusSearch;
bool normalizeBins;
double EPSAngle;
double maxCurv;

std::vector<double> pointSize;

void initParams(ros::NodeHandle n) {
    n.getParam("/radiusSearch", normalRadiusSearch);
    n.getParam("/normalizeBins", normalizeBins);
    n.getParam("/EPSAngle", EPSAngle);
    n.getParam("/maxCurv", maxCurv);

    n.getParam("/whatTypeOfData", whatTypeOfData);

    n.getParam("/pointSize", pointSize);

    views_dir = ros::package::getPath("train_classifier") + "/Data/Views/" + whatTypeOfData;
    vfhs_dir = ros::package::getPath("train_classifier") + "/Data/VFHS/" + whatTypeOfData;
    training_dir = ros::package::getPath("classifier") + "/Data/Training/" + whatTypeOfData;
}

void loadVFHModels(std::vector<vfh_model> & models, std::string color) {
    // TODO: Use the one in cvfh instead!
    for (size_t i = 0; i < objects[color].size(); i++) {
        size_t j = 0;
        while (true) {
            std::string dir = vfhs_dir + "/" + color + "/" + objects[color][i] + "/" + boost::lexical_cast<std::string>(j) + vfhs_extension;

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

            std::vector<pcl::PCLPointField> fields;
            pcl::getFieldIndex(point, "vhf", fields);

            for (size_t l = 0; l < point.points.size(); l++) {
                vfh_model vfh;
                vfh.second.resize(308);

                for (size_t k = 0; k < fields[pcl::getFieldIndex(cloud, "vfh")].count; k++) {
                    vfh.second[k] = point.points[l].histogram[k];
                }
                vfh.first = objects[color][i]; // Maybe something else so we know the pose?

                models.push_back(vfh);
            }

            j++;
        }
    }
}

void loadViews(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> & views, std::string color, std::string object) {
    boost::filesystem::path p(views_dir + "/" + color + "/" + object);

    for (boost::filesystem::directory_iterator i = boost::filesystem::directory_iterator(p); i != boost::filesystem::directory_iterator(); i++) {
        if (!boost::filesystem::is_directory(i->path())) {
            std::string fileName = i->path().filename().string();

            if (fileName.find(view_extension) != std::string::npos) {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

                pcl::io::loadPCDFile<pcl::PointXYZRGB>(views_dir + "/" + color + "/" + object + "/" + fileName, *cloud);

                views.push_back(cloud);
            }
        }
    }
}

void train() {
    for (std::map<std::string, std::vector<std::string> >::iterator iter = objects.begin(); iter != objects.end(); iter++) {
        std::string color = iter->first;

        for (size_t i = 0; i < objects[color].size(); i++) {
            std::cout << "Processing [" << (i+1) << "/" << objects[color].size() << "]: " << color << " " << objects[color][i] << std::endl;

            // Create directory
            boost::filesystem::path dir(vfhs_dir + "/" + color + "/" + objects[color][i]);
            boost::filesystem::remove_all(dir); // Remove first so we can add new!
            boost::filesystem::create_directories(dir);

            std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> views;

            loadViews(views, color, objects[color][i]);

            // For each view calculate CVFH
            for (size_t j = 0; j < views.size(); j++) {
                std::cout << "Processing view " << (j+1) << " of " << views.size() << std::endl;
                pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308>);

                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
                cloud_xyz->points.resize(views[j]->size());
                for (size_t p = 0; p < cloud_xyz->points.size(); p++) {
                    cloud_xyz->points[p].x = views[j]->points[p].x;
                    cloud_xyz->points[p].y = views[j]->points[p].y;
                    cloud_xyz->points[p].z = views[j]->points[p].z;
                }

                VFHHelper::computeVFHSignature(vfhs, cloud_xyz, normalRadiusSearch, normalizeBins, EPSAngle, maxCurv);

                // Save VFH signature to file
                // TODO: Maybe save pose too?!
                // Save and use the views name instead?!
                pcl::io::savePCDFileBinary(vfhs_dir + "/" + color + "/" + objects[color][i] + "/" + boost::lexical_cast<std::string>(j) + vfhs_extension, *vfhs);
            }
        }

        // Load all histograms of all models
        std::vector<vfh_model> models;
        loadVFHModels(models, color);

        // Convert data into FLANN format
        flann::Matrix<float> data (new float[models.size() * models[0].second.size()], models.size(), models[0].second.size());

        for (size_t i = 0; i < data.rows; i++) {
            for (size_t j = 0; j < data.cols; j++) {
                data[i][j] = models[i].second[j];
            }
        }

        // Save data to disk (list of models)
        boost::filesystem::path dir(training_dir + "/" + color);
        boost::filesystem::remove_all(dir); // Remove first so we can add new!
        boost::filesystem::create_directories(dir);
        flann::save_to_file (data, training_dir + "/" + color + "/" + training_data_h5_file_name, "training_data");

        std::ofstream fs;
        fs.open ((training_dir + "/" + color + "/" + training_data_list_file_name).c_str());
        for (size_t i = 0; i < models.size (); i++) {
          fs << models[i].first << "\n";
        }
        fs.close ();

        // Build the tree index and save it to disk
        flann::Index<flann::ChiSquareDistance<float> > index (data, flann::LinearIndexParams());
        //flann::Index<flann::ChiSquareDistance<float> > index (data, flann::KDTreeIndexParams (4));    // Faster but only approximative

        index.buildIndex();
        index.save(training_dir + "/" + color + "/" + kdtree_idx_file_name);
    }
}

void getObjects() {
    boost::filesystem::path p(views_dir);

    for (boost::filesystem::directory_iterator i = boost::filesystem::directory_iterator(p); i != boost::filesystem::directory_iterator(); i++) {
        std::string color = i->path().filename().string();
        if (boost::filesystem::is_directory(i->path())) {
            for (boost::filesystem::directory_iterator j = boost::filesystem::directory_iterator(i->path()); j != boost::filesystem::directory_iterator(); j++) {
                if (boost::filesystem::is_directory(j->path())) {
                    std::string type = j->path().filename().string();

                    objects[color].push_back(type);
                    /*
                    if (objects.count(color)) {
                        objects[color].push_back(type);
                    } else {
                        std::vector<std::string> temp;
                        temp.push_back(type);
                        objects[color] = temp;
                    }
                    */
                }
            }
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
