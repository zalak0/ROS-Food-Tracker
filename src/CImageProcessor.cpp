#include "turtlebot3_gazebo/CImageProcessor.hpp"


/********************************************************************************
** Constructor and Destructors
********************************************************************************/
CImageProcessor::CImageProcessor()
: Node("image_processor")
{
    // Create a subscriber to the camera image topic
    mSubImage = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10, std::bind( &CImageProcessor::ProcessImageCallback, this, std::placeholders::_1 ));

    // Create a publisher to send commands based on image analysis
    mPubImageStatus = this->create_publisher<std_msgs::msg::String>( "/image_status", 10 );

    RCLCPP_INFO( this->get_logger(), "Maze image processor node has been initialised" );
}


/********************************************************************************
** Helper Functions
********************************************************************************/
double CImageProcessor::GetMatchPercentage( const sensor_msgs::msg::Image::SharedPtr msg )
{
    // Convert the ROS image message to an OpenCV image
    cv_bridge::CvImagePtr pCv;

    try
    {
        pCv = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8 );
    }
    catch( cv_bridge::Exception& e )
    {
        RCLCPP_ERROR( this->get_logger(), "cv_bridge exception: %s", e.what() );
        return 0.0;
    }

    // Convert the BGR image to HSV
    cv::Mat hsvImage;
    cv::cvtColor( pCv->image, hsvImage, cv::COLOR_BGR2HSV );

    // Define the target color range in HSV with tolerance for #25F956 (Greenish color)
    cv::Scalar lowerHsv( 40, 100, 100 );  // Lower bound for hue, saturation, and value
    cv::Scalar upperHsv( 80, 255, 255 );  // Upper bound for hue, saturation, and value

    // Create a mask where the color matches
    cv::Mat mask;
    cv::inRange( hsvImage, lowerHsv, upperHsv, mask );

    // Calculate the percentage of matching pixels
    double matchPercentage = (double) cv::countNonZero( mask ) / ( hsvImage.rows * hsvImage.cols );
    
    RCLCPP_INFO( this->get_logger(), "Match percentage: %.2f%%", matchPercentage * 100.0 );
    return matchPercentage;
}


/********************************************************************************
** Callback Functions
********************************************************************************/
void CImageProcessor::ProcessImageCallback( const sensor_msgs::msg::Image::SharedPtr msg )
{
    double matchPercentage = GetMatchPercentage( msg );
    std_msgs::msg::String imageStatusMsg;

    // Send move command if match percentage is between 30% and 55%
    if(( matchPercentage >= mLowerThreshold ) && ( matchPercentage < mUpperThreshold ))
    {
        imageStatusMsg.data = "move_forward";
        RCLCPP_INFO( this->get_logger(), "Sending move forward command." );
    }
    // Send stop command if match percentage is greater than or equal to 55%
    else if( matchPercentage >= mUpperThreshold )
    {
        imageStatusMsg.data = "stop";
        RCLCPP_INFO( this->get_logger(), "Sending stop command." );
    }
    // If match percentage is below 30%, do nothing or follow default wall-following behavior
    else
    {
        imageStatusMsg.data = "default";
        RCLCPP_INFO( this->get_logger(), "No significant match, following default behavior." );
    }

    // Publish the status message
    mPubImageStatus->publish( imageStatusMsg );
}


/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char **argv)
{
    rclcpp::init( argc, argv );
    rclcpp::spin( std::make_shared<CImageProcessor>() );
    rclcpp::shutdown();
    return 0;
}
