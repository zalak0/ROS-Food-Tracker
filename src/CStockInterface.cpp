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
    std::string command;
    std::string item;

    std::cout << "==== Welcome to the Stock System Interface ====" << std::endl
              << "There is 1 command available:" << std::endl
              << " 1. stock: <id-or-name>?" << std::endl << std::endl;

    while( 1 )
    {
        std::cin >> command;
        std::cin >> item;
        
        ParseInput( command + ' ' + item );
        
        command = "";
        item = "";

        std::cout << std::endl << "Ready for next command:" << std::endl;
    }
}


// Parses user input into the appropriate function calls
void CStockInterface::ParseInput( std::string aUserInput )
{
    std::string command;
    std::string item;

    std::stringstream ss( aUserInput );
    ss >> command;
    ss >> item;

    // Check item is valid
    int markerId = atoi( item.c_str() );

    if( markerId == 0 )
    {
        std::cout << "Error: invalid item id. Check it is a positive integer." << std::endl;
    }

    // Check command is valid
    else if( command == "stock:" )  
    {
        std::cout << "There are " << std::to_string( CheckQuantity( markerId ) )
                  << " items (SKU: " << markerId << ") in stock." << std::endl;
    }
    else
    {
        std::cout << "Error: command not recognised. Check your spelling." << std::endl;
    }
}


// Prints the quantity of the specified item to the command line
int CStockInterface::CheckQuantity( int aMarkerId )
{
    return mpStockList->GetQuantity( aMarkerId );
}
