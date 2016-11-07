#include <ros/ros.h>

#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>

#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>


#include <string>

#include <pcl/features/cvfh.h>
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

std::vector<std::string> objects;

std::string save_dir = ros::package::getPath("train_classifier") + "/Data/Views";
std::string data_extension = ".pcd";

std::string models_dir = ros::package::getPath("generate_views_cad") + "/Data/Models";
std::string model_extension = ".ply";

int resolution;
float viewAngle;
int tesselationLevel;

void initParams(ros::NodeHandle n) {
    n.getParam("/resolution", resolution);
    n.getParam("/viewAngle", viewAngle);
    n.getParam("/tesselationLevel", tesselationLevel);
}

void getViews(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & views, std::string objectName) {
    std::string objectLocation = models_dir + "/" + objectName + model_extension;

    // Load the PLY model from a file.
    vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
    reader->SetFileName(objectLocation.c_str());
    reader->Update();

    // VTK is not exactly straightforward...
    vtkSmartPointer < vtkPolyDataMapper > mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(reader->GetOutputPort());
    mapper->Update();

    vtkSmartPointer<vtkPolyData> object = mapper->GetInput();

    // Virtual scanner object.
    pcl::apps::RenderViewsTesselatedSphere render_views;
    render_views.addModelFromPolyData(object);
    // Pixel width of the rendering window, it directly affects the snapshot file size.
    render_views.setResolution(resolution);
    // Horizontal FoV of the virtual camera.
    render_views.setViewAngle(viewAngle);
    // If true, the resulting clouds of the snapshots will be organized.
    render_views.setGenOrganized(false);
    // How much to subdivide the icosahedron. Increasing this will result in a lot more snapshots.
    render_views.setTesselationLevel(tesselationLevel);
    // If true, the camera will be placed at the vertices of the triangles. If false, at the centers.
    // This will affect the number of snapshots produced (if true, less will be made).
    // True: 42 for level 1, 162 for level 2, 642 for level 3...
    // False: 80 for level 1, 320 for level 2, 1280 for level 3...
    render_views.setUseVertices(false);
    // If true, the entropies (the amount of occlusions) will be computed for each snapshot (optional).
    //render_views.setComputeEntropies(true);
    //render_views.setRadiusSphere(1.0f); // What is this?!

    render_views.generateViews();

    // Object for storing the rendered views.
    //std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> views;
    // Object for storing the poses, as 4x4 transformation matrices.
    //std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;
    // Object for storing the entropies (optional).
    //std::vector<float> entropies;
    render_views.getViews(views);
    //render_views.getPoses(poses);
    //render_views.getEntropies(entropies);
}

void generateViews() {
    // Get views of all models
    for (size_t i = 0; i < objects.size(); i++) {
        std::cout << "Processing: " << objects[i] << std::endl;
        // Make dir for this object
        boost::filesystem::path dir(save_dir + "/CAD/" + objects[i]);
        boost::filesystem::remove_all(dir); // Remove first so we can add new!
        boost::filesystem::create_directories(dir);

        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> views;

        getViews(views, objects[i]);

        // Save all views
        for (size_t j = 0; j < views.size(); j++) {
            pcl::io::savePCDFileASCII(save_dir + "/CAD/" + objects[i] + "/" + boost::lexical_cast<std::string>(j) + data_extension, *views[j]);
        }
    }
}

void getObjects() {
    // Check folder for .ply
    boost::filesystem::path p(models_dir);

    for (boost::filesystem::directory_iterator i = boost::filesystem::directory_iterator(p); i != boost::filesystem::directory_iterator(); i++) {
        if (!boost::filesystem::is_directory(i->path())) {
            std::string fileName = i->path().filename().string();

            if (fileName.find(model_extension) != std::string::npos) {
                objects.push_back(fileName.substr(0, fileName.size() - model_extension.size()));
            }
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "generate_views_cad");
    ros::NodeHandle nh;

    initParams(nh);

    getObjects();

    generateViews();
}
