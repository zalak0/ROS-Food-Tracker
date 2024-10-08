#include "turtlebot3_gazebo/CDriveLogic.hpp"

#include <memory>

using namespace std::chrono_literals;


/************************************************************
** Constructors and Destructors
************************************************************/
CDriveLogic::CDriveLogic()
: Node( "turtlebot3_drive_node" )
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
    RCLCPP_INFO( this->get_logger(), "Turtlebot3 simulation node has been terminated" );
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
    uint16_t scan_angle[ mNumScans ] = {90, 0};

    for( int i=0; i<mNumScans; i++ )
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
    double targetDist = 0.5;  // Target distance the bot maintains from the wall
    double minDist = 0.3;     // Minimum distance to object in front of bot

    if( mGoalReached )
    {
        RCLCPP_INFO( this->get_logger(), "Goal reached, stopping the robot." );
        UpdateVelocityCommand( 0.0, 0.0 );
        return;
    }
    else if (mSeekGoal)
    {
        UpdateVelocityCommand( mLinearVelocity, 0.0 );
    }
    else if (mScanData[FRONT] < minDist)
    {
        UpdateVelocityCommand( 0.0, -1 * mAngularVelocity );
    }
    else if (mScanData[LEFT] < targetDist)
    {
        UpdateVelocityCommand( mLinearVelocity, -0.3 * mAngularVelocity );
    }
    else if (mScanData[LEFT] > targetDist)
    {
        UpdateVelocityCommand( mLinearVelocity, 0.3 * mAngularVelocity );
    }
    else
    {
        UpdateVelocityCommand( mLinearVelocity, 0.0 );
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
