#include "turtlebot3_gazebo/CStockList.hpp"

#include <iostream>


CStockList::CStockList()
: mFileName( "stock" ),
  mItems()
{   
    #if TEST == false
    auto qos = rclcpp::QoS( rclcpp::KeepLast(10) );

    // Initialise publishers
    mPubStockUpdate = this->create_publisher<std_msgs::msg::String>( "stock_update", qos );

    // Initialise subscribers
    mScannedMarkerId = this->create_subscription<std_msgs::msg::String>(
        "scanned_marker_id", qos, std::bind( &CStockList::RecordStockCallback, this, std::placeholders::_1 ));
    #endif
}


CStockList::CStockList( std::string aFilePath )
: mFileName( aFilePath ),
  mItems()
{   
    #if TEST == false
    auto qos = rclcpp::QoS( rclcpp::KeepLast(10) );

    // Initialise publishers
    mPubStockUpdate = this->create_publisher<std_msgs::msg::String>( "stock_update", qos );

    // Initialise subscribers
    mScannedMarkerId = this->create_subscription<std_msgs::msg::String>(
        "scanned_marker_id", qos, std::bind( &CStockList::RecordStockCallback, this, std::placeholders::_1 ));
    #endif
}


CStockList::~CStockList()
{
    for (std::unordered_map< int, CItem* >::iterator it = mItems.begin(); it != mItems.end(); ++it) {
        delete it->second;
    }

    mItems.clear();
}


int CStockList::GetQuantity( int aMarkerId )
{
    return mItems[ aMarkerId ]->GetQuantity();
}


void CStockList::RenameItem( int aMarkerId, std::string aName )
{
    mItems[ aMarkerId ]->SetName( aName );
}


void CStockList::WriteToDisk()
{   
    std::cout << "Stock list exported to " << mFileName << ".txt" << std::endl;
    return;
}


#if TEST == false
void CStockList::RecordStockCallback( const std_msgs::msg::Integer::SharedPtr msg )
{   
    item = RetrieveItem( msg->data );

    if( item )
    {
        item.IncrementQuantity();
    }
    else
    {
        CreateItem( msg.data );
    }
}
#endif


void CStockList::CreateItem( int aMarkerId )
{
    mItems[ aMarkerId ] = new CItem( aMarkerId );
}


CItem* CStockList::RetrieveItem( int aMarkerId )
{
    std::unordered_map< int, CItem* >::iterator it = mItems.find( aMarkerId );

    if( it != mItems.end() )
    {
        return it->second;
    }
    else
    {
        return NULL;
    }
}


void CStockList::UpdateItem( int aMarkerId, int aQuantity, std::string aName )
{   
    if( RetrieveItem( aMarkerId ))
    {   
        mItems[ aMarkerId ]->SetName( aName );
        mItems[ aMarkerId ]->SetQuantity( aQuantity );
    }
}


void CStockList::DeleteItem( int aMarkerId )
{
    delete RetrieveItem( aMarkerId );
    mItems.erase( aMarkerId );
}


void CStockList::LoadFile( std::string aFilePath )
{
    return;
}
