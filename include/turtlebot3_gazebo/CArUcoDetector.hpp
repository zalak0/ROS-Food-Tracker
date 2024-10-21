#ifndef TURTLEBOT3_GAZEBO__ARUCO_DETECTOR
#define TURTLEBOT3_GAZEBO__ARUCO_DETECTOR

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

/********************************************************************************
** CArUcoDetector: The class which analyses the frames published by the camera
**                  driver, publishing the detected ArUco markers.
********************************************************************************/
class CArUcoDetector : public rclcpp::Node
{
public:
    CArUcoDetector();

    // Declare the method here
    int DetectArUcoTagsAndReturnID(const sensor_msgs::msg::Image::SharedPtr msg);

private:
    // Constants
    const double mLowerThreshold = 0.3;
    const double mUpperThreshold = 0.55;

    // ProcessImageCallback: Processes the image and detects ArUco markers.
    void ProcessImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    // ROS topic subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mSubImage;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mPubImageStatus;
};

#endif // TURTLEBOT3_GAZEBO__ARUCO_DETECTOR
