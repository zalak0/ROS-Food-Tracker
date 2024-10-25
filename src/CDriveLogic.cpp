#include "turtlebot3_gazebo/CDriveLogic.hpp"
#include <memory>

using namespace std::chrono_literals;

/************************************************************
** Constructors and Destructors
************************************************************/
CDriveLogic::CDriveLogic()
: Node("turtlebot3_drive_node")
{
    for (int i = 0; i < mNumScans; i++)
    {
        mScanData[i] = 0.0;
    }

    mGoalReached = false;
    mSeekGoal = false;

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    // Initialise publishers
    mPubCommandVelocity = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

    // Initialise subscribers
    mSubScan = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(), std::bind(&CDriveLogic::ScanCallback, this, std::placeholders::_1));

    mSubMazeSolved = this->create_subscription<std_msgs::msg::String>(
        "/image_status", qos, std::bind(&CDriveLogic::ImageStatusCallback, this, std::placeholders::_1));

    mUpdateTimer = this->create_wall_timer(8ms, std::bind(&CDriveLogic::GetCommandCallback, this));

    RCLCPP_INFO(this->get_logger(), "Turtlebot3 simulation node has been initialised with Left Wall Following");
}


CDriveLogic::~CDriveLogic()
{
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    mPubCommandVelocity->publish(cmd_vel);

    RCLCPP_INFO(this->get_logger(), "Turtlebot3 simulation node has been terminated");
}


/************************************************************
** Subscriber Callbacks
************************************************************/
void CDriveLogic::ImageStatusCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if (msg->data == "move_forward")
    {
        mSeekGoal = true;
        mGoalReached = false;
    }
    else if (msg->data == "stop")
    {
        mGoalReached = true;
        mSeekGoal = false;
    }
    else // Default behavior
    {   
        mGoalReached = false;
        mSeekGoal = false;
    }

    RCLCPP_INFO(this->get_logger(), "Image status received: %s", msg->data.c_str());
}


void CDriveLogic::ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    uint16_t scan_angle[mNumScans] = {90, 0};  // Left = 90 degrees, Front = 0 degrees

    for (int i = 0; i < mNumScans; i++)
    {
        if (std::isinf(msg->ranges.at(scan_angle[i])))
        {
            mScanData[i] = msg->range_max;
        }
        else
        {
            mScanData[i] = msg->ranges.at(scan_angle[i]);
        }
    }

    // Log distances for debugging
    RCLCPP_INFO(this->get_logger(), "Front Distance: %.2f, Left Distance: %.2f", mScanData[FRONT], mScanData[LEFT]);
}


void CDriveLogic::UpdateVelocityCommand(double linear, double angular)
{
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = linear;
    cmd_vel.angular.z = angular;

    mPubCommandVelocity->publish(cmd_vel);
}


/********************************************************************************
** Timer Callback - Left Wall Following Algorithm with Obstacle Avoidance
********************************************************************************/
void CDriveLogic::GetCommandCallback()
{
    double targetDist = 0.4;   // Target distance from the left wall (40 cm)
    double minDist = 0.2;      // Minimum distance to object in front (30 cm)
    double tooCloseDist = 0.15; // Too-close distance to initiate stronger right turn (20 cm)
    double linearSpeed = 0.15;  // Default linear speed
    double angularSpeed = 0.15; // Default angular speed for turning

    // Obstacle avoidance - If there's an obstacle in front, turn right
    if (mScanData[FRONT] < minDist) 
    {
        // Turn right to avoid the obstacle
        UpdateVelocityCommand(0.05, -angularSpeed * 2.0);  // Slower speed, stronger right turn
        RCLCPP_INFO(this->get_logger(), "Obstacle in front, turning right.");
        return;
    }

    // Wall following logic on the left side
    if (mScanData[LEFT] < tooCloseDist) // Too close to left wall
    {
        // Move away from the left wall by making a sharper right turn
        UpdateVelocityCommand(0.05, -angularSpeed * 1.5);  // Slower speed, stronger right turn
        RCLCPP_WARN(this->get_logger(), "Too close to left wall, moving away.");
    }
    else if (mScanData[LEFT] > targetDist + 0.1) // Too far from the left wall
    {
        // Adjust to move closer to the left wall (turn left slightly)
        UpdateVelocityCommand(linearSpeed, angularSpeed);
    }
    else if (mScanData[LEFT] < targetDist - 0.1) // Slightly too close to left wall
    {
        // Adjust to move away from the wall (turn right slightly)
        UpdateVelocityCommand(linearSpeed, -angularSpeed);
    }
    else
    {
        // Move forward while maintaining alignment with the left wall
        UpdateVelocityCommand(linearSpeed, 0.0);
    }

    // Log the robot's movement details for debugging
    RCLCPP_INFO(this->get_logger(), "Linear Vel: %.2f, Angular Vel: %.2f", linearSpeed, angularSpeed);
}


/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CDriveLogic>());
    rclcpp::shutdown();

    return 0;
}
