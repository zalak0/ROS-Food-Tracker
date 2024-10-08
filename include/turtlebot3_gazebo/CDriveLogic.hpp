#ifndef TURTLEBOT3_GAZEBO__DRIVE_LOGIC
#define TURTLEBOT3_GAZEBO__DRIVE_LOGIC

#include "std_msgs/msg/string.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp> 
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>


/********************************************************************************
** Enumerators
********************************************************************************/
enum eDirections
{
    LEFT = 0,
    FRONT
};


/********************************************************************************
** CDriveLogic: The class implementing the logic determining where the turtlebot
**              drives based on scan (LiDAR) data and the end zone detection
**              status indicated by the image processing node.
********************************************************************************/
class CDriveLogic : public rclcpp::Node
{
public:
    CDriveLogic();
    ~CDriveLogic();

private:
    // ROS topic publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mPubCommandVelocity; // To send velocity commands 

    // ROS topic subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr mSubScan;  // To receive scan data
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mSubMazeSolved;  // To receive image status

    // Constants
    static const int mNumScans = 2;         // The number of scan ranges to store
    const double mLinearVelocity = 0.25;    // Linear velocity to travel at
    const double mAngularVelocity = 1.5;    // Angular velocity to travel at

    // Variables
    double mScanData[mNumScans];            // The most recently received scan ranges
    bool mGoalReached;                      // Whether the end zone has been reached
    bool mSeekGoal;                         // Whether the end zone has been sighted

    // ROS timer
    rclcpp::TimerBase::SharedPtr mUpdateTimer;

    // GetCommandCallback: Computes the current velocity command.
    void GetCommandCallback();
    
    // UpdateVelocityCommand: Updates the velocity setpoint of the turtlebot diff
    //                        drive to the specified values.
    // Parameters:
    // - linear: Linear velocity to move at (m/s)
    // - angular: Angular velocity to move at (rad/s)
    void UpdateVelocityCommand( double linear, double angular );

    // ScanCallback: Loads the new scan data into the member variable.
    // Parameters:
    // - msg: message received from the LiDAR sensor
    void ScanCallback( const sensor_msgs::msg::LaserScan::SharedPtr msg );
    
    // ImageStatusCallback: Sets navigation parameters based on the status
    //                      received from the image processor node.
    // Parameters:
    // - msg: message received from the image processor node
    void ImageStatusCallback( const std_msgs::msg::String::SharedPtr msg );
};

#endif  // TURTLEBOT3_GAZEBO__DRIVE_LOGIC
