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

int iLowH = 1;      // 0
int iHighH = 10;    // 179

int iLowS = 138;    // 0
int iHighS = 255;   // 255

int iLowV = 130;      // 0
int iHighV = 255;   // 255

int iLastX = -1;
int iLastY = -1;

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {

}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat imgOriginal = cv_bridge::toCvShare(msg, "bgr8")->image;

    cv::Mat imgHSV;

    cv::cvtColor(imgOriginal, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    cv::Mat imgThresholded;

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

    double x = m10 / area;
    double y = m01 / area;

    //std::cout << "X: " << x << "\tY: " << y << std::endl;

    double height = msg->height / 3;
    double width = msg->width / 3;

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

    //std::cout << "X1: " << mu[0].m10 / mu[0].m00 << ", Y1: " << mu[0].m01 / mu[0].m00 << "\t\tX2: " << mu[1].m10 / mu[1].m00 << ", Y2: " << mu[1].m01 / mu[1].m00 << std::endl;
    cv::imshow("Thresholded Image", imgThresholded); //show the thresholded image
    cv::imshow("Original", imgOriginal); //show the original image

    if (cv::waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
    {
        std::cout << "esc key is pressed by user" << std::endl;
    }
}

int main(int argc, char **argv)
{
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
  image_transport::Subscriber sub = it.subscribe("camera/rgb/image_raw", 1, imageCallback);

  ros::spin();


}
