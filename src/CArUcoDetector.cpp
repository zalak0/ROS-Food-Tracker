#include "turtlebot3_gazebo/CArUcoDetector.hpp"


// Constructor for CArUcoDetector
CArUcoDetector::CArUcoDetector()
: Node("aruco_detector")
{
    // Create a subscriber to the camera image topic
    mSubImage = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10, std::bind(&CArUcoDetector::ProcessImageCallback, this, std::placeholders::_1));

    // Create a publisher to send the detected ArUco markers
    mPubScannedMarkerId = this->create_publisher<turtlebot3_gazebo::msg::AprilTag>("scanned_marker_id", 10);

    RCLCPP_INFO(this->get_logger(), "ArUco image processor node initialized");
}

// Detect ArUco tags
void CArUcoDetector::detectTags(const sensor_msgs::msg::Image::SharedPtr msg, std::vector<std::pair< int, int >>* scannedTags)
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

    if (markerIds.empty())
    {
        RCLCPP_INFO(this->get_logger(), "No ArUco markers detected.");
    }
    
    for( int i=0; i<markerIds.size(); i++)
    {
        std::pair< int,int > pair = {markerIds[i], markerCorners[i].x};
        scannedTags->push_back(pair);

        RCLCPP_INFO(this->get_logger(), "Detected ArUco ID: %d, at: %d", markerIds[i], markerCorners[i].x);
    }
    
    return ;
}

// Callback to process image and detect ArUco markers
void CArUcoDetector::ProcessImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    std::vector<std::pair< int, int >> scannedTags;
    detectTags( msg, &scannedTags );

    for(int i=0; i<scannedTags.size(); i++)
    {
        turtlebot3_gazebo::msg::AprilTag scannedTagMsg;
        scannedTagMsg.id = scannedTags[i].first;
        scannedTagMsg.pixel = scannedTags[i].second;
        mPubScannedMarkerId->publish(scannedTagMsg);
    }

    scannedTags.clear();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CArUcoDetector>());
    rclcpp::shutdown();
    return 0;
}
