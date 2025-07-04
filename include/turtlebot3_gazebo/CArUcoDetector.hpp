#ifndef TURTLEBOT3_GAZEBO__ARUCO_DETECTOR
#define TURTLEBOT3_GAZEBO__ARUCO_DETECTOR

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "turtlebot3_gazebo/msg/april_tag.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"  // Updated to handle compressed images
#include "sensor_msgs/image_encodings.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <utility>

/********************************************************************************
** CArUcoDetector: The class which analyses the frames published by the camera
**                 driver, publishing the detected ArUco markers.
********************************************************************************/
class CArUcoDetector : public rclcpp::Node
{
public:
    CArUcoDetector();

    // Detects any ArUco tags in an image
    void detectTags(const sensor_msgs::msg::CompressedImage::SharedPtr msg,
                    std::vector<std::pair<int, double>> scannedTags);

private:
    // ProcessImageCallback: Processes the image and detects ArUco markers.
    void ProcessImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

    // ROS topic subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr mSubImage;  // Updated to compressed image
    rclcpp::Publisher<turtlebot3_gazebo::msg::AprilTag>::SharedPtr mPubScannedMarkerId;
};

#endif // TURTLEBOT3_GAZEBO__ARUCO_DETECTOR
