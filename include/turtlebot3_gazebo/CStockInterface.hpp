#ifndef STOCK_INTERFACE
#define STOCK_INTERFACE

#include "turtlebot3_gazebo/CStockList.hpp"


/********************************************************************************
** CStockInterface: The object which users interact with to view and modify 
**                  stock.
********************************************************************************/
class CStockInterface
{
public:
    CStockInterface();

    CStockInterface( CStockList* apStockList );

    ~CStockInterface();

    #if TEST == true
    void StockUpdateCallback( std::string aUpdateMsg );
    #endif
    
private:
    CStockList* mpStockList;

    #if TEST == false
    // ROS topic subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mSubStockUpdate;

    // ROS subscriber callback prints stock list changes to the command line
    void StockUpdateCallback( const std_msgs::msg::String::SharedPtr msg );
    #endif

    // Waits for user input via the command line
    void WaitOnInput();

    // Parses user input into the appropriate function calls
    void ParseInput( std::string aUserInput );

    // Prints the quantity of the specified item to the command line
    int CheckQuantity( int aMarkerId );
};

#endif