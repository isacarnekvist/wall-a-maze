#include <ros/ros.h>
#include " sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "opencv2/imgproc/imgproc.hpp"

int iLowH = 0;
int iHighH = 179;

int iLowS = 0;
int iHighS = 255;

int iLowV = 0;
int iHighV = 255;

bool image = true;

cv::Mat imgOriginal;

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {

}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if (!image) {
        return;
    }
    image = false;
    imgOriginal = cv_bridge::toCvShare(msg, "bgr8")->image;

    /*
    cv::Mat imgHSV;

    cv::cvtColor(imgOriginal, imgHSV, cv::COLOR_BGR2HSV);

    cv::Mat imgThresholded;

    cv::inRange (imgHSV, cv::Scalar())

  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image );
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  */
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

  while (ros::ok ()) {
      if (!image) {
      cv::Mat imgHSV;

      cv::cvtColor(imgOriginal, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

      cv::Mat imgThresholded;

      cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

      //morphological opening (remove small objects from the foreground)
      cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
      cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

      //morphological closing (fill small holes in the foreground)
      cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
      cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

      cv::imshow("Thresholded Image", imgThresholded); //show the thresholded image
      cv::imshow("Original", imgOriginal); //show the original image

      if (cv::waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
             {
                  std::cout << "esc key is pressed by user" << std::endl;

             }
      }
      ros::spinOnce ();
  }


}
