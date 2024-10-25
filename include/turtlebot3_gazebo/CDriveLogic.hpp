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
    FRONT,
};

/********************************************************************************
** Constants
********************************************************************************/
const int mNumScans = 2;  // Number of scan directions (FRONT, LEFT)

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
    void ImageStatusCallback(const std_msgs::msg::String::SharedPtr msg);
    void ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void UpdateVelocityCommand(double linear, double angular);
    void GetCommandCallback();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mPubCommandVelocity;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr mSubScan;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mSubMazeSolved;
    rclcpp::TimerBase::SharedPtr mUpdateTimer;

    double mScanData[mNumScans];  // Holds scan data (FRONT, LEFT)
    bool mGoalReached;
    bool mSeekGoal;

    // New member variables for distance tracking and turning logic
    double mDistanceTraveled;
    double mPrevScanRange;
    bool mPerformTurn;
    double mTurnAngle;

    const double mLinearVelocity = 0.2;   // Example linear velocity
    const double mAngularVelocity = 0.5;  // Example angular velocity
};
#endif  // TURTLEBOT3_GAZEBO__DRIVE_LOGIC
