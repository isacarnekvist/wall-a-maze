#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "geometry_msgs/PointStamped.h"
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

#include <pcl_ros/point_cloud.h>

#include <pcl/filters/approximate_voxel_grid.h>

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
ros::Publisher object_pub;

void HSVFilter(pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_out, double hMin, double hMax, double sMin, double sMax, double vMin, double vMax) {
    /*
    pcl::PointIndices::Ptr indices (new pcl::PointIndices ());
    for (size_t i = 0; i < cloud_in.points.size(); i++) {
        if (cloud_in.points[i].h >= hMin && cloud_in.points[i].h <= hMax && cloud_in.points[i].s >= sMin && cloud_in.points[i].s <= sMax && cloud_in.points[i].v >= vMin && cloud_in.points[i].v <= vMax) {
            indices->indices.push_back(i);
        }
    }
    */

    pcl::PassThrough<pcl::PointXYZHSV> pass;

    pass.setInputCloud (cloud_in);
    pass.setFilterFieldName ("h");
    pass.setFilterLimits (hMin, hMax);
    pass.setFilterLimitsNegative (false);
    pass.filter (*cloud_in);

    pass.setInputCloud (cloud_in);
    pass.setFilterFieldName ("s");
    pass.setFilterLimits (sMin, sMax);
    pass.setFilterLimitsNegative (false);
    pass.filter (*cloud_in);

    pass.setInputCloud (cloud_in);
    pass.setFilterFieldName ("s");
    pass.setFilterLimits (sMin, sMax);
    pass.setFilterLimitsNegative (false);
    pass.filter (*cloud_out);
}

void pointCloudCallback(const pcl::PointCloud<pcl::PointXYZHSV>::ConstPtr& input_cloud) {

    // Filtering input scan to increase speed of registration.
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZHSV>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZHSV> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
    approximate_voxel_filter.setInputCloud (input_cloud);
    approximate_voxel_filter.filter (*filtered_cloud);


    pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZHSV>);
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZHSV>);
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZHSV>);


    // Transform
    const Eigen::Vector3f translation(0.0, -0.152, 0.115);

    double yaw = 0.0;
    //double roll = -30.0 * PI / 180.0;
    double roll = -42.0 * PI / 180.0;
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

    pcl::transformPointCloud(*filtered_cloud, *cloud, translation, rotation);


    // Filter on color
    // Red
    int iLowH = 1;      // 0
    int iHighH = 10;    // 179

    int iLowS = 138;    // 0
    int iHighS = 255;   // 255

    int iLowV = 130;      // 0
    int iHighV = 255;   // 255

    /*
    pcl::ConditionAnd<pcl::PointXYZHSV>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZHSV> ());
      color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZHSV>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZHSV> ("h", pcl::ComparisonOps::LT, iHighH)));
      color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZHSV>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZHSV> ("h", pcl::ComparisonOps::GT, iLowH)));
      color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZHSV>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZHSV> ("s", pcl::ComparisonOps::LT, iHighS)));
      color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZHSV>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZHSV> ("s", pcl::ComparisonOps::GT, iLowS)));
      color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZHSV>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZHSV> ("v", pcl::ComparisonOps::LT, iHighV)));
      color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZHSV>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZHSV> ("v", pcl::ComparisonOps::GT, iLowV)));

    pcl::ConditionalRemoval<pcl::PointXYZHSV> condrem(color_cond);
    condrem.setInputCloud(cloud);
    condrem.setKeepOrganized(true);

    condrem.filter(*cloud_filtered);
    */

    HSVFilter(cloud, cloud_filtered, iLowH, iHighH, iLowS, iHighS, iLowV, iHighV);


    // Filter so only top remains

    pcl::PointXYZHSV min_p, max_p;

    pcl::getMinMax3D(*cloud_filtered, min_p, max_p);

    //std::cout << "Min: x=" << min_p.x << " y=" << min_p.y << " z=" << min_p.z << std::endl << "Max: x" << max_p.x << " y=" << max_p.y << " z=" << max_p.z << std::endl;


    pcl::PassThrough<pcl::PointXYZHSV> pass;
    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (min_p.y - 0.015, min_p.y + 0.015);
    pass.filter(*cloud_final);

    sensor_msgs::PointCloud2 output;

    pcl::toROSMsg(*cloud_filtered, output);


    // Calculate optimal pick up position
    // PCL -> RIKTIG
    // z = x
    // x = y

    double forward = 0.0;
    double side = 0.0;
    double height = 0.0;

    double numPoints = 0.0;
    for (pcl::PointCloud<pcl::PointXYZHSV>::iterator point = cloud_final->points.begin(); point < cloud_final->points.end(); point++) {
        numPoints++;

        forward += point->z;
        side += point->x;
        height -= point->y; // Inverted
    }

    std::cout << "Forward: " << forward / numPoints << "\tSide: " << side / numPoints << "\tHeight: " << height / numPoints << std::endl;

    pub.publish(output);

    geometry_msgs::PointStamped pointStamped;
    std_msgs::Header pheader;
    //pheader.stamp = inf; // not set!
    pheader.frame_id = "wheel_center";
    pointStamped.point.x = forward / numPoints;
    pointStamped.point.y = side / numPoints;
    pointStamped.point.z = height / numPoints;

    object_pub.publish(pointStamped);

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

  object_pub = nh.advertise<geometry_msgs::PointStamped> ("objectPos_wheelcenter", 1);

  ros::spin();


}
