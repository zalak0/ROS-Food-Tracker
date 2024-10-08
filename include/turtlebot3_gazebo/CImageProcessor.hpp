#ifndef TURTLEBOT3_GAZEBO__IMAGE_PROCESSOR
#define TURTLEBOT3_GAZEBO__IMAGE_PROCESSOR

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

#include <opencv2/opencv.hpp>


/********************************************************************************
** CImageProcessor: The class which analyses the frames published by the camera
**                  driver, publishing whether the correct colour in a sufficient
**                  quantity has been detected (indicates the maze end-zone).
********************************************************************************/
class CImageProcessor : public rclcpp::Node
{
public:
    CImageProcessor();

private:
    // Constants
    const double mLowerThreshold = 0.3;  // Minimum percentage of pixels required for maze to be considered solved
    const double mUpperThreshold = 0.55;  // Minimum percentage of pixels required for maze to be considered solved

    // ProcessImageCallback: Calculate what percent of pixels lie in a particular
    //                       colour range representing the maze end-zone.
    // Parameters:
    // - msg: message containing image to analyse
    double GetMatchPercentage( const sensor_msgs::msg::Image::SharedPtr msg );

    // ProcessImageCallback: Publish the current image status based on the most
    //                       recent image recieved from the camera driver.
    // Parameters:
    // - msg: message received from image topic
    void ProcessImageCallback( const sensor_msgs::msg::Image::SharedPtr msg );

    // ROS topic subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mSubImage;     // Topic containing frames sent from turtlebot3 camera
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mPubImageStatus;    // Topic indicating whether the maze is solved
};

#endif // TURTLEBOT3_GAZEBO__IMAGE_PROCESSOR