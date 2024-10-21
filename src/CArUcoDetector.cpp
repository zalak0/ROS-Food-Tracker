#include "turtlebot3_gazebo/CArUcoDetector.hpp"


// Constructor for CArUcoDetector
CArUcoDetector::CArUcoDetector()
: Node("aruco_detector")
{
    // Create a subscriber to the camera image topic
    mSubImage = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10, std::bind(&CArUcoDetector::ProcessImageCallback, this, std::placeholders::_1));

    // Create a publisher to send the detected ArUco markers
    mPubScannedMarkerId = this->create_publisher<std_msgs::msg::Int32>("scanned_marker_id", 10);

    RCLCPP_INFO(this->get_logger(), "ArUco image processor node initialized");
}

// Detect ArUco tags and return their IDs
int CArUcoDetector::DetectArUcoTagsAndReturnID(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImagePtr pCv;

    try {
        pCv = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGRA8);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return -1;  // Return -1 if conversion fails
    }

    if (pCv->image.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Empty image received from the camera.");
        return -1;
    }

    cv::Mat grayImage;
    cv::cvtColor(pCv->image, grayImage, cv::COLOR_BGR2GRAY);

    if (grayImage.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to convert image to grayscale.");
        return -1;
    }

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_16h5);
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;

    // Detect ArUco markers
    cv::aruco::detectMarkers(grayImage, dictionary, markerCorners, markerIds);

    if (markerIds.empty()) {
        RCLCPP_INFO(this->get_logger(), "No ArUco markers detected.");
        return -1;
    }

    int detectedId = markerIds[0];

    // Log and display the detected markers
    RCLCPP_INFO(this->get_logger(), "Detected ArUco ID: %d", detectedId);
    cv::waitKey(1);

    return detectedId;
}

// Callback to process image and detect ArUco markers
void CArUcoDetector::ProcessImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    static std::mutex wordMutex;  
    int detectedId = -1;

    {
        std::lock_guard<std::mutex> lock(wordMutex);

        detectedId = DetectArUcoTagsAndReturnID(msg);
    }

    if (detectedId != -1) {
        std_msgs::msg::Int32 scannedTagMsg;
        scannedTagMsg.data = detectedId;
        mPubScannedMarkerId->publish(scannedTagMsg);
    }    
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CArUcoDetector>());
    rclcpp::shutdown();
    return 0;
}
