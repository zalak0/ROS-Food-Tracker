#include "turtlebot3_gazebo/CStockList.hpp"


CStockList::CStockList()
: mFileName( "stock" ),
  mItems()
{   
    #if TEST == false
    auto qos = rclcpp::QoS( rclcpp::KeepLast(10) );

    // Initialise publishers
    mPubStockUpdate = this->create_publisher<std_msgs::msg::String>( "stock_update", qos );

    // Initialise subscribers
    mSubScannedMarkerId = this->create_subscription<std_msgs::msg::String>(
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
    mSubScannedMarkerId = this->create_subscription<std_msgs::msg::String>(
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
    static const int numColumns = 3;
    static const int columnWidths[ numColumns ] = {10, 10, 20};

    // Create and open file
    std::string line;
    std::string filePath = "../bin/" + mFileName + ".txt";
    std::ofstream file( filePath );

    if ( !file.is_open() )
    { 
        std::cout << "Error: file could not be created." << std::endl;
    }

    std::ios_base& left( std::ios_base& str );

    // Write file header
    file << std::left
         << std::setw( columnWidths[ 0 ] ) << "SKU" << " | "
         << std::setw( columnWidths[ 1 ] ) << "Quantity" << " | "
         << std::setw( columnWidths[ 2 ] ) << "Name" << std::endl
         << std::string( columnWidths[ 0 ], '-' ) << "-|-"
         << std::string( columnWidths[ 1 ], '-' ) << "-|-"
         << std::string( columnWidths[ 2 ], '-' ) << std::endl;

    // Write items to file
    for (std::unordered_map< int, CItem* >::iterator it = mItems.begin(); it != mItems.end(); ++it) {
        file << std::left
             << std::setw( columnWidths[ 0 ] ) << it->first << " | "
             << std::setw( columnWidths[ 1 ] ) << it->second->GetQuantity() << " | "
             << std::setw( columnWidths[ 2 ] ) << it->second->GetName() << std::endl;
    }

    std::cout << "Stock list successfully exported to " << mFileName << ".txt" << std::endl;

    return;
}


#if TEST == false
void CStockList::RecordStockCallback( const std_msgs::msg::Integer::SharedPtr msg )
{
    CItem* item = RetrieveItem( msg->data );

    if( item )
    {
        item->IncrementQuantity();
        SendUpdate( QUANTITY, aMarkerId );
    }
    else
    {
        CreateItem( msg->data );
    }
}
#else
void CStockList::TestRecordStockCallback( int aMarkerId )
#endif
{
    CItem* item = RetrieveItem( aMarkerId );

    if( item )
    {
        item->IncrementQuantity();
        SendUpdate( QUANTITY, aMarkerId );
    }
    else
    {
        CreateItem( aMarkerId );
    }
}


void CStockList::SendUpdate( eUpdateType aUpdateType, int aMarkerId )
{   
    std::string updateMsg = "Stock Update: Item (SKU: " + std::to_string( aMarkerId ) + ") ";
    std::string quantity = std::to_string( RetrieveItem( aMarkerId )->GetQuantity() );
    std::string name = RetrieveItem( aMarkerId )->GetName();

    switch( aUpdateType )
    {
        case CREATE:    updateMsg += "created.\n"; break;
        case DELETE:    updateMsg += "deleted.\n"; break;
        case QUANTITY:  updateMsg += "quantity changed to " + quantity + "\n"; break;
        case RENAME:    updateMsg += "renamed to " + name + "\n"; break;
        default:        updateMsg += "did not update.\n";
    }

    std::cout << updateMsg;

    #if TEST == false
    std_msgs::msg::String stockUpdate;
    mPubStockUpdate->publish( stockUpdate );
    #endif
}


void CStockList::CreateItem( int aMarkerId )
{
    mItems[ aMarkerId ] = new CItem( aMarkerId );
    SendUpdate( CREATE, aMarkerId );
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
    CItem* item = RetrieveItem( aMarkerId );

    if( item && item->GetQuantity() != aQuantity )
    {   
        mItems[ aMarkerId ]->SetQuantity( aQuantity );
        SendUpdate( QUANTITY, aMarkerId );
    }

    if( item && item->GetName() != aName )
    {   
        mItems[ aMarkerId ]->SetName( aName );
        SendUpdate( RENAME, aMarkerId );
    }
}


void CStockList::DeleteItem( int aMarkerId )
{
    delete RetrieveItem( aMarkerId );
    mItems.erase( aMarkerId );
    SendUpdate( DELETE, aMarkerId );
}


void CStockList::LoadFile( std::string aFilePath )
{
    return;
}
