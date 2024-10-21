#include "turtlebot3_gazebo/CArUcoDetector.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/string.hpp>
#include <mutex>  // For thread safety
#include <string> // For std::to_string

CArUcoDetector::CArUcoDetector()
: Node("aruco_detector")
{
    // Create a subscriber to the camera image topic
    mSubImage = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10, std::bind(&CArUcoDetector::ProcessImageCallback, this, std::placeholders::_1));

    // Create a publisher to send the number-based word based on detected tags
    mPubImageStatus = this->create_publisher<std_msgs::msg::String>("/image_status", 10);

    RCLCPP_INFO(this->get_logger(), "Maze image processor node has been initialised");
}

// Helper function to detect ArUco tags and return their IDs as integers
int CArUcoDetector::DetectArUcoTagsAndReturnID(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // Convert the ROS image message to an OpenCV image
    cv_bridge::CvImagePtr pCv;

    try
    {
        pCv = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return -1;  // Return -1 if conversion fails
    }

    // Check if the image is empty
    if (pCv->image.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Empty image received from the camera.");
        return -1;
    }

    // Convert the BGR image to grayscale (ArUco detection works better in grayscale)
    cv::Mat grayImage;
    cv::cvtColor(pCv->image, grayImage, cv::COLOR_BGR2GRAY);

    if (grayImage.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to convert image to grayscale.");
        return -1;
    }

    // --- ArUco detection setup ---
    // You can use other dictionaries such as DICT_4X4_50 or DICT_4X4_100 if needed.
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;

    // Detect ArUco markers in the image
    cv::aruco::detectMarkers(grayImage, dictionary, markerCorners, markerIds);

    // If no markers are found, return -1
    if (markerIds.empty())
    {
        RCLCPP_INFO(this->get_logger(), "No ArUco markers detected.");
        return -1;
    }

    // Assuming we are only interested in the first detected ArUco marker's ID
    int detectedId = markerIds[0];

    // Log the detected ID
    RCLCPP_INFO(this->get_logger(), "Detected ArUco ID: %d", detectedId);

    // Draw the detected markers on the image for debugging
    cv::aruco::drawDetectedMarkers(pCv->image, markerCorners, markerIds);

    // Display the result (optional, for debugging purposes)
    cv::imshow("Detected ArUco Markers", pCv->image);
    cv::waitKey(1);

    return detectedId;
}

// Callback function to process the received image and detect ArUco markers
void CImageProcessor::ProcessImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    static std::string numberString = "";  // Holds the current number string
    static std::mutex wordMutex;           // Mutex to ensure thread safety

    {
        std::lock_guard<std::mutex> lock(wordMutex);

        // Detect ArUco tag and get the numeric ID
        int detectedId = DetectArUcoTagsAndReturnID(msg);

        // Append the numeric ID to the string if a valid marker ID was detected
        if (detectedId != -1)
        {
            numberString += std::to_string(detectedId);  // Convert the ID to a string and append it
            RCLCPP_INFO(this->get_logger(), "Current Number String: %s", numberString.c_str());
        }
    }

    // Publish the current number string as the status message
    std_msgs::msg::String imageStatusMsg;
    imageStatusMsg.data = numberString;
    mPubImageStatus->publish(imageStatusMsg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CImageProcessor>());
    rclcpp::shutdown();
    return 0;
}