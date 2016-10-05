#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <stdio.h>
#include <opencv2/opencv.hpp>
# define PI           3.14159265358979323846  /* pi */

ros::Subscriber rgb_sub;

void initParams(ros::NodeHandle n) {
    // Robot
    //n.getParam("/robot/wheel_radius", wheel_radius);
    //n.getParam("/robot/wheel_base", wheel_base);

    // Other
    //n.getParam("/general/control_frequency", control_frequency);
    //n.getParam("/general/ticks_per_rev", ticks_per_rev);
}

// How we wish to move
void rgbCallback(const sensor_msgs::Image::ConstPtr& msg) {
    ROS_INFO("Width: %d, Height: %d, Encoding: %s, Step: %u, Data size: %lu, Data[0]: %u", msg->width, msg->height, msg->encoding.c_str(), msg->step, msg->data.size(), msg->data[0]);
    /*
    {
        using namespace cv;

        Mat image;
        image = imdecode( msg->data, 1 );

        if ( !image.data )
        {
            printf("No image data \n");
            return;
        }
        namedWindow("Display Image", WINDOW_AUTOSIZE );
        imshow("Display Image", image);

        waitKey(0);
    }
    */

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "perception");

    ros::NodeHandle n;

    initParams(n);

    rgb_sub = n.subscribe("/camera/rgb/image_raw", 1000, rgbCallback);

    ros::Rate loop_rate(125); // TODO: USE VARIABLE IN GENERAL

    while (ros::ok()) {
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
