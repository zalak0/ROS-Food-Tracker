#include "turtlebot3_gazebo/CDriveLogic.hpp"
#include <memory>
#include <cmath>  // For M_PI

using namespace std::chrono_literals;

/************************************************************
** Constructors and Destructors
************************************************************/
CDriveLogic::CDriveLogic()
: Node( "turtlebot3_drive_node" ),
  mDistanceTraveled(0.0),
  mPrevScanRange(0.0),
  mPerformTurn(false),
  mTurnAngle(0.0)
{
    for( int i = 0; i < mNumScans; i++ )
    {
        mScanData[i] = 0.0;
    }

    mGoalReached = false;
    mSeekGoal = false;

    auto qos = rclcpp::QoS( rclcpp::KeepLast(10) );

    // Initialise publishers
    mPubCommandVelocity = this->create_publisher<geometry_msgs::msg::Twist>( "cmd_vel", qos );

    // Initialise subscribers
    mSubScan = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(), std::bind( &CDriveLogic::ScanCallback, this, std::placeholders::_1 ));

    mSubMazeSolved = this->create_subscription<std_msgs::msg::String>(
        "/image_status", qos, std::bind( &CDriveLogic::ImageStatusCallback, this, std::placeholders::_1 ));

    mUpdateTimer = this->create_wall_timer( 8ms, std::bind( &CDriveLogic::GetCommandCallback, this ));

    RCLCPP_INFO( this->get_logger(), "Turtlebot3 simulation node has been initialised" );
}

CDriveLogic::~CDriveLogic()
{
    // Create a Twist message to stop the robot
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;

    // Publish the stop command
    mPubCommandVelocity->publish(cmd_vel);

    // Add a small delay to ensure the message is processed before shutdown
    rclcpp::sleep_for(500ms);

    RCLCPP_INFO(this->get_logger(), "Turtlebot3 simulation node has been terminated and robot stopped.");
}


/************************************************************
** Subscriber Callbacks
************************************************************/
void CDriveLogic::ImageStatusCallback( const std_msgs::msg::String::SharedPtr msg )
{
    if( msg->data == "move_forward" )
    {
        mSeekGoal = true;
        mGoalReached = false;
    }
    else if( msg->data == "stop" )
    {
        mGoalReached = true;
        mSeekGoal = false;
    }
    else // Default behavior
    {   
        mGoalReached = false;
        mSeekGoal = false;
    }

    RCLCPP_INFO( this->get_logger(), "Image status received: %s", msg->data.c_str() );
}


void CDriveLogic::ScanCallback( const sensor_msgs::msg::LaserScan::SharedPtr msg )
{
    uint16_t scan_angle[ mNumScans ] = {90, 0};  // Left = 90 degrees, Front = 0 degrees

    for( int i = 0; i < mNumScans; i++ )
    {
        if( std::isinf( msg->ranges.at( scan_angle[ i ] )))
        {
            mScanData[ i ] = msg->range_max;
        }
        else
        {
            mScanData[ i ] = msg->ranges.at( scan_angle[ i ] );
        }
    }
}


void CDriveLogic::UpdateVelocityCommand( double linear, double angular )
{
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = linear;
    cmd_vel.angular.z = angular;

    mPubCommandVelocity->publish( cmd_vel );
}


/********************************************************************************
** Timer Callback
********************************************************************************/
void CDriveLogic::GetCommandCallback()
{
    double targetDist = 0.1;  // Target distance the bot maintains from the wall
    double minDist = 0.3;     // Minimum distance to object in front of bot
    double distanceStep = 0.3; // Distance to travel before stopping (30cm)

    if( mGoalReached )
    {
        RCLCPP_INFO( this->get_logger(), "Goal reached, stopping the robot." );
        UpdateVelocityCommand( 0.0, 0.0 );
        return;
    }

    // Calculate distance traveled based on laser scan (simple approximation)
    if (!mPerformTurn)
    {
        double currentRange = mScanData[FRONT];
        if (mPrevScanRange > 0) 
        {
            double deltaRange = std::abs(currentRange - mPrevScanRange);
            mDistanceTraveled += deltaRange;
        }
        mPrevScanRange = currentRange;

        // Check if the robot has traveled the required distance
        if (mDistanceTraveled >= distanceStep) 
        {
            // Stop and prepare to perform a 360-degree turn
            UpdateVelocityCommand(0.0, 0.0);
            mPerformTurn = true;
            mTurnAngle = 0.0;
            mDistanceTraveled = 0.0;  // Reset the traveled distance
        }
    }

    // Perform the 360-degree turn
    if (mPerformTurn) 
    {
        double angularTurnSpeed = M_PI / 2.0; // Speed of turning (half a radian per second)
        mTurnAngle += angularTurnSpeed * 0.008; // 8ms timer update
        UpdateVelocityCommand(0.0, angularTurnSpeed);

        // Check if the turn is complete
        if (mTurnAngle >= 2 * M_PI) 
        {
            // Turn completed, stop and resume forward movement
            UpdateVelocityCommand(0.0, 0.0);
            mPerformTurn = false;
        }
        return;
    }

    // Regular driving behavior if no turn is needed
    if (mScanData[FRONT] < minDist)
    {
        UpdateVelocityCommand(0.0, -1 * mAngularVelocity);
    }
    else if (mScanData[LEFT] < targetDist)
    {
        UpdateVelocityCommand(mLinearVelocity, -0.3 * mAngularVelocity);
    }
    else if (mScanData[LEFT] > targetDist)
    {
        UpdateVelocityCommand(mLinearVelocity, 0.3 * mAngularVelocity);
    }
    else
    {
        UpdateVelocityCommand(mLinearVelocity, 0.0);
    }
}


/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char **argv)
{
    rclcpp::init( argc, argv );
    rclcpp::spin( std::make_shared<CDriveLogic>() );
    rclcpp::shutdown();

    return 0;
}
