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
    void Nav2CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg); // NEW: Nav2 cmd_vel callback
    void UpdateVelocityCommand(double linear, double angular);
    void GetCommandCallback();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mPubCommandVelocity;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr mSubScan;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mSubMazeSolved;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mSubNav2CmdVel;  // NEW: Subscribe to Nav2 cmd_vel
    rclcpp::TimerBase::SharedPtr mUpdateTimer;

    double mScanData[mNumScans];  // Holds scan data (FRONT, LEFT)
    bool mGoalReached;
    bool mSeekGoal;

    // New member variables for distance tracking and turning logic
    double mDistanceTraveled;    // Accumulated total distance traveled from the start
    double mLastStopDistance;    // Distance at the last stop point
    double mPrevScanRange;       // Previous scan range for calculating the delta
    bool mPerformTurn;           // Flag to check if the robot is in turn mode
    double mTurnAngle;           // Current turn angle during a 360-degree turn

    // NEW: Hold Nav2 velocity command
    double mNav2LinearVel;
    double mNav2AngularVel;

    const double mLinearVelocity = 0.1;   // Example linear velocity
    const double mAngularVelocity = 0.2;  // Example angular velocity
};
#endif  // TURTLEBOT3_GAZEBO__DRIVE_LOGIC
