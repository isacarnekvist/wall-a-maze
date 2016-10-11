#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <tf/transform_datatypes.h>

#define PI           3.14159265358979323846  /* pi */

/*
// RED
int iLowH = 1;      // 0
int iHighH = 10;    // 179

int iLowS = 138;    // 0
int iHighS = 255;   // 255

int iLowV = 130;      // 0
int iHighV = 255;   // 255
*/

// GREEN
int iLowH = 33;      // 0
int iHighH = 66;    // 179

int iLowS = 90;    // 0
int iHighS = 255;   // 255

int iLowV = 77;      // 0
int iHighV = 255;   // 255

ros::Publisher pub;

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    //std::cout << "Height: " << msg->height << "\tWidth: " << msg->width << std::endl;
    /*
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr cloud_pass(new pcl::PCLPointCloud2);

    pcl_conversions::toPCL(*msg, *cloud);
    */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::fromROSMsg(*msg, *cloud_tmp);

    //Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    //(0.115, 0.0, 0.202),
    //        tf.transformations.quaternion_from_euler(0.0, 30.0 * pi / 180.0, 0.0),

    const Eigen::Vector3f translation(0.0, -0.202, 0.115);

    double yaw = 0.0;
    double roll = -30.0 * PI / 180.0;
    double pitch = 0.0;

    double t0 = std::cos(yaw * 0.5f);
    double t1 = std::sin(yaw * 0.5f);
    double t2 = std::cos(roll * 0.5f);
    double t3 = std::sin(roll * 0.5f);
    double t4 = std::cos(pitch * 0.5f);
    double t5 = std::sin(pitch * 0.5f);

    double w = t0 * t2 * t4 + t1 * t3 * t5;
    double x = t0 * t3 * t4 - t1 * t2 * t5;
    double y = t0 * t2 * t5 + t1 * t3 * t4;
    double z = t1 * t2 * t4 - t0 * t3 * t5;

    std::cout << "w: " << w << "\tx: " << x << "\ty: " << y << "\tz: " << z << std::endl;

    Eigen::Quaternionf rotation(w, x, y, z);

    pcl::transformPointCloud(*cloud_tmp, *cloud, translation, rotation);

    // GREEN
    int rMax = 120;
    int rMin = 0;
    int gMax = 255;
    int gMin = 100;
    int bMax = 70;
    int bMin = 0;

    // RED
    /*
    int rMax = 255;
    int rMin = 110;
    int gMax = 80;
    int gMin = 0;
    int bMax = 35;
    int bMin = 0;
    */
    // PURPLE
    /*
    int rMax = 160;
    int rMin = 80;
    int gMax = 130;
    int gMin = 60;
    int bMax = 180;
    int bMin = 120;
    */
    // BLUE
    /*
    int rMax = 50;
    int rMin = 0;
    int gMax = 150;
    int gMin = 60;
    int bMax = 255;
    int bMin = 90;
    */
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
      color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, rMax)));
      color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, rMin)));
      color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LT, gMax)));
      color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, gMin)));
      color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LT, bMax)));
      color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, bMin)));

    pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem(color_cond);
    condrem.setInputCloud(cloud);
    condrem.setKeepOrganized(true);

    condrem.filter(*cloud_filtered);

    pcl::PointXYZRGB min_p, max_p;

    pcl::getMinMax3D(*cloud_filtered, min_p, max_p);

    std::cout << "Min: x=" << min_p.x << " y=" << min_p.y << " z=" << min_p.z << std::endl << "Max: x" << max_p.x << " y=" << max_p.y << " z=" << max_p.z << std::endl;


    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (min_p.y - 0.015, min_p.y + 0.015);
    pass.filter(*cloud_final);

    /*
    pcl::PassThrough<pcl::PCLPointCloud2> pass;
    pass.setInputCloud(cloudPtr);
    pass.setFilterFieldName("r");
    pass.setFilterLimits(0.0, 10);
    pass.filter(*cloud_pass);

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud_pass);
    sor.setLeafSize(0.002, 0.002, 0.002);
    sor.filter(*cloud_filtered);


    pcl_conversions::fromPCL(*cloud_filtered, output);
    */

    sensor_msgs::PointCloud2 output;

    pcl::toROSMsg(*cloud_final, output);

    // PCL -> RIKTIG
    // z = x
    // x = y

    double forward;
    double side;

    double numPoints = 0.0;
    for (pcl::PointCloud<pcl::PointXYZRGB>::iterator point = cloud_final->points.begin(); point < cloud_final->points.end(); point++) {
        numPoints++;

        forward += point->z;
        side += point->x;
    }

    std::cout << "Forward: " << forward / numPoints << "\tSide: " << side / numPoints << std::endl;

    pub.publish(output);

}

cv::Mat imgThresholded;
double x = -1;
double y = -1;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv::Mat imgOriginal = cv_bridge::toCvShare(msg, "bgr8")->image;

    cv::Mat imgHSV;

    cv::cvtColor(imgOriginal, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

/*
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(imgThresholded, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    std::vector<cv::Moments> mu(contours.size());
    for (int i = 0; i < contours.size(); i++) {
        mu[i] = cv::moments(contours[i], false);
    }
*/
    cv::Moments moments = cv::moments(imgThresholded, false);

    /*
    double m01 = mu[0].m01;
    double m10 = mu[0].m10;
    double area = mu[0].m00;
    */
    double m01 = moments.m01;
    double m10 = moments.m10;
    double area = moments.m00;

    x = m10 / area;
    y = m01 / area;

    //std::cout << "X: " << x << "\tY: " << y << std::endl;

    double height = msg->height / 3;
    double width = msg->width / 3;
    /*
    if (y < height) {
        // Top
        std::cout << "Top ";
    } else if (y < height * 2) {
        // Middle
        std::cout << "Middle ";
    } else {
        // Bottom
        std::cout << "Bottom ";
    }
    if (x < width) {
        // Left
        std::cout << "Left" << std::endl;
    } else if (x < width * 2) {
        // Middle
        std::cout << "Center" << std::endl;
    } else {
        // Right
        std::cout << "Right" << std::endl;
    }
    */
    //std::cout << "X1: " << mu[0].m10 / mu[0].m00 << ", Y1: " << mu[0].m01 / mu[0].m00 << "\t\tX2: " << mu[1].m10 / mu[1].m00 << ", Y2: " << mu[1].m01 / mu[1].m00 << std::endl;
    cv::imshow("Thresholded Image", imgThresholded); //show the thresholded image
    cv::imshow("Original", imgOriginal); //show the original image

    if (cv::waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
    {
        std::cout << "esc key is pressed by user" << std::endl;
    }
}

void depthImageCallback(const sensor_msgs::ImageConstPtr& msg) {
    if (x == -1 || y == -1) {
        return;
    }

    cv::Mat imgOriginal = cv_bridge::toCvShare(msg)->image;

    std::cout << imgOriginal.at<uint16_t>(msg->width/2.0, msg->height/2.0) / 10.0 << " cm" << std::endl;

    cv::imshow("Depth", imgOriginal); //show the original image

    if (cv::waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
    {
        std::cout << "esc key is pressed by user" << std::endl;
    }

    //std::cout << msg->encoding << std::endl;
    //std::cout << "Size: " << msg->data.size() << "\tHeight: " << msg->height << "\tWidth: " << msg->width << std::endl;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;


  cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"



  //Create trackbars in "Control" window
  cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
  cvCreateTrackbar("HighH", "Control", &iHighH, 179);

  cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
  cvCreateTrackbar("HighS", "Control", &iHighS, 255);

  cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
  cvCreateTrackbar("HighV", "Control", &iHighV, 255);


  image_transport::ImageTransport it(nh);
  //image_transport::Subscriber image_sub = it.subscribe("camera/rgb/image_raw", 1, imageCallback);
  //image_transport::Subscriber depth_sub = it.subscribe("camera/depth/image_raw", 1, depthImageCallback);
  ros::Subscriber point_sub = nh.subscribe("camera/depth_registered/points", 1, pointCloudCallback);

  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  ros::spin();


}
