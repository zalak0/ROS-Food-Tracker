#include "turtlebot3_gazebo/CImageProcessor.hpp"
#include <mutex>  // For thread safety

CImageProcessor::CImageProcessor()
: Node("image_processor")
{
    // Create a subscriber to the camera image topic
    mSubImage = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10, std::bind(&CImageProcessor::ProcessImageCallback, this, std::placeholders::_1));

    // Create a publisher to send the ASCII word based on detected tags
    mPubImageStatus = this->create_publisher<std_msgs::msg::String>("/image_status", 10);

    RCLCPP_INFO(this->get_logger(), "Maze image processor node has been initialised");
}

// Helper function to detect ArUco tags and convert their IDs to ASCII characters
char CImageProcessor::DetectArUcoTagsAndConvertToASCII(const sensor_msgs::msg::Image::SharedPtr msg)
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
        return '\0';  // Return null character if conversion fails
    }

    // Convert the BGR image to grayscale (ArUco detection works better in grayscale)
    cv::Mat grayImage;
    cv::cvtColor(pCv->image, grayImage, cv::COLOR_BGR2GRAY);

    // --- ArUco detection setup ---
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;

    // Detect ArUco markers in the image
    cv::aruco::detectMarkers(grayImage, dictionary, markerCorners, markerIds);

    // If no markers are found, return null character
    if (markerIds.empty())
    {
        RCLCPP_INFO(this->get_logger(), "No ArUco markers detected.");
        return '\0';
    }

    // Assuming we are only interested in the first detected ArUco marker's ID
    int detectedId = markerIds[0];

    // --- Convert detected ArUco marker ID to ASCII ---
    if (detectedId < 32 || detectedId > 126)  // Only valid for printable ASCII characters (32-126)
    {
        RCLCPP_WARN(this->get_logger(), "Marker ID %d is not a printable ASCII character.", detectedId);
        return '\0';
    }

    char asciiChar = static_cast<char>(detectedId);
    RCLCPP_INFO(this->get_logger(), "Detected ArUco ID: %d, ASCII Character: %c", detectedId, asciiChar);

    return asciiChar;
}

void CImageProcessor::ProcessImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    static std::string asciiWord = "";  // Holds the current ASCII word
    static std::mutex wordMutex;        // Mutex to ensure thread safety

    {
        // Lock the mutex to ensure thread-safe access to the asciiWord
        std::lock_guard<std::mutex> lock(wordMutex);

        // Detect ArUco tag and get the ASCII character
        char asciiChar = DetectArUcoTagsAndConvertToASCII(msg);

        // Append the character to the word if a valid ASCII character was detected
        if (asciiChar != '\0')
        {
            asciiWord += asciiChar;
            RCLCPP_INFO(this->get_logger(), "Current ASCII Word: %s", asciiWord.c_str());
        }
    }

    // Publish the current word as the status message
    std_msgs::msg::String imageStatusMsg;
    imageStatusMsg.data = asciiWord;
    mPubImageStatus->publish(imageStatusMsg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CImageProcessor>());
    rclcpp::shutdown();
    return 0;
}
