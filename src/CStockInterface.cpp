#include "turtlebot3_gazebo/CStockInterface.hpp"


CStockInterface::CStockInterface()
: mpStockList( new CStockList() )
{
    WaitOnInput();
}


CStockInterface::CStockInterface( CStockList* apStockList )
: mpStockList( apStockList )
{
    WaitOnInput();
}


CStockInterface::~CStockInterface()
{
    delete mpStockList;
}


#if TEST == true
void CStockInterface::StockUpdateCallback( std::string aUpdateMsg )
{
    std::cout << aUpdateMsg;
}
#else
// ROS subscriber callback prints stock list changes to the command line
void CStockInterface:: StockUpdateCallback( const std_msgs::msg::String::SharedPtr msg )
{
    std::cout << aUpdateMsg;
}
#endif


// Waits for user input via the command line
void CStockInterface::WaitOnInput()
{   
    std::string query;
    std::string userInput;

    std::cout << "Welcome to the Stock System Interface" << std::endl
              << "There is 1 command are available:" << std::endl
              << " 1. How many <id-or-name> are in stock?" << std::endl;

    while( 1 )
    {
        std::cin >> userInput;
        query += userInput + ' ';
        
        if( userInput.back() == '?' )
        {
            ParseInput( query );
            query = "";
        }
    }
}


// Parses user input into the appropriate function calls
void CStockInterface::ParseInput( std::string aUserInput )
{
    std::cout << "Command Received: " << aUserInput << std::endl;
}


// Prints the quantity of the specified item to the command line
int CStockInterface::CheckQuantity( int aMarkerId )
{
    return mpStockList->GetQuantity( aMarkerId );
}
