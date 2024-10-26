#include "turtlebot3_gazebo/CArUcoDetector.hpp"
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// Constructor for CArUcoDetector
CArUcoDetector::CArUcoDetector()
: Node("aruco_detector")
{
    // Create a subscriber to the compressed camera image topic
    mSubImage = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/image_raw/compressed", 10, std::bind(&CArUcoDetector::ProcessImageCallback, this, std::placeholders::_1));

    // Create a publisher to send the detected ArUco markers
    mPubScannedMarkerId = this->create_publisher<turtlebot3_gazebo::msg::AprilTag>("scanned_marker_id", 10);

    RCLCPP_INFO(this->get_logger(), "ArUco image processor node initialized for compressed images");
}

// Detect ArUco tags
void CArUcoDetector::detectTags(const sensor_msgs::msg::CompressedImage::SharedPtr msg, std::vector<std::pair<int, int>>* scannedTags)
{
    cv::Mat decodedImage;
    
    try {
        // Decode the compressed image
        // Using OpenCV function cv::imdecode to decode the compressed image into a cv::Mat format
        decodedImage = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Check if the decoding process was successful
    if (decodedImage.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Empty image received from the camera.");
        return;
    }

    // Convert the color image to grayscale for easier processing
    cv::Mat grayImage;
    cv::cvtColor(decodedImage, grayImage, cv::COLOR_BGR2GRAY);

    if (grayImage.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to convert image to grayscale.");
        return;
    }

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_16h5);
    std::vector<int> markerIds;  
    std::vector<std::vector<cv::Point2f>> markerCorners;  

    // Detect ArUco markers in the grayscale image
    cv::aruco::detectMarkers(grayImage, dictionary, markerCorners, markerIds);

    if (markerIds.empty())
    {
        RCLCPP_INFO(this->get_logger(), "No ArUco markers detected.");
    }
    
    for (int i = 0; i < markerIds.size(); i++)
    {
        std::pair<int, int> pair = {markerIds[i], static_cast<int>(markerCorners[i][0].x)};
        scannedTags->push_back(pair);

        RCLCPP_INFO(this->get_logger(), "Detected ArUco ID: %d, at: %d", markerIds[i], static_cast<int>(markerCorners[i][0].x));
    }
}

// Create a cooldown period after an image is scanned to avoid double scan 
void CArUcoDetector::cooldown()
{
    std::this_thread::sleep_for(std::chrono::seconds(10));
}

// Callback to process image and detect ArUco markers
void CArUcoDetector::ProcessImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    std::vector<std::pair<int, int>> scannedTags;
    detectTags(msg, &scannedTags);

    for (int i = 0; i < scannedTags.size(); i++)
    {
        turtlebot3_gazebo::msg::AprilTag scannedTagMsg;
        scannedTagMsg.id = scannedTags[i].first;
        scannedTagMsg.pixel = scannedTags[i].second;
        mPubScannedMarkerId->publish(scannedTagMsg);
    }

    scannedTags.clear();

    std::thread cooldownThread(&CArUcoDetector::cooldown, this);
    cooldownThread.detach();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CArUcoDetector>());
    rclcpp::shutdown();
    return 0;
}