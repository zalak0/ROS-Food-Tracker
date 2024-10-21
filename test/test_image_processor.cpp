#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <path_to_image>" << std::endl;
        return -1;
    }

    // Load the image from the file
    std::string imagePath = argv[1];
    cv::Mat image = cv::imread(imagePath);

    if (image.empty()) {
        std::cerr << "Failed to load image: " << imagePath << std::endl;
        return -1;
    }

    // Convert the image to grayscale
    cv::Mat grayImage;
    cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);

    // Test with 4x4 dictionaries (50, 100, 250, 1000)
    std::vector<cv::Ptr<cv::aruco::Dictionary>> dictionaries = {
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50),
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100),
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250),
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000)
    };

    std::vector<std::string> dictionaryNames = {
        "DICT_4X4_50", "DICT_4X4_100", "DICT_4X4_250", "DICT_4X4_1000"
    };

    for (size_t i = 0; i < dictionaries.size(); ++i) {
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;

        // Detect the ArUco markers
        cv::aruco::detectMarkers(grayImage, dictionaries[i], markerCorners, markerIds);

        // Check if any markers were detected
        if (markerIds.empty()) {
            std::cout << "No ArUco markers detected with dictionary " << dictionaryNames[i] << std::endl;
        } else {
            // Log the detected marker IDs
            std::cout << "Detected ArUco IDs with dictionary " << dictionaryNames[i] << ": ";
            for (int id : markerIds) {
                std::cout << id << " ";
            }
            std::cout << std::endl;

            // Draw the detected markers on the image
            cv::aruco::drawDetectedMarkers(image, markerCorners, markerIds);

            // Display the image with detected markers
            cv::imshow("Detected ArUco Markers", image);
            cv::waitKey(0); // Wait for any key press
        }
    }

    return 0;
}
