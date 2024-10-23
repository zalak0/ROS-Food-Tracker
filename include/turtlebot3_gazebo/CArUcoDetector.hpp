#ifndef TURTLEBOT3_GAZEBO__ARUCO_DETECTOR
#define TURTLEBOT3_GAZEBO__ARUCO_DETECTOR

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <mutex>

/********************************************************************************
** CArUcoDetector: The class which analyses the frames published by the camera
**                 driver, publishing the detected ArUco markers.
********************************************************************************/
class CArUcoDetector : public rclcpp::Node
{
public:
    CArUcoDetector();

    // Detects any ArUco tags in an iamge
    int DetectArUcoTagsAndReturnID(const sensor_msgs::msg::Image::SharedPtr msg);

private:
    // ProcessImageCallback: Processes the image and detects ArUco markers.
    void ProcessImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    // ROS topic subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mSubImage;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mPubScannedMarkerId;
};

#endif // TURTLEBOT3_GAZEBO__ARUCO_DETECTOR
