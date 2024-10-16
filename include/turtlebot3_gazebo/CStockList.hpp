#ifndef STOCK_LIST
#define STOCK_LIST

#include <std_msgs/msg/integer.hpp> 
#include <std_msgs/msg/string.hpp> 

#include <string>
#include <unordered_map>


//---Forward Declarations------------------------------------------------------
class CItem;


//---Enumerators---------------------------------------------------------------

enum eUpdateType
{
    CREATE,
    DELETE,
    QUANTITY,
    RENAME
};


/********************************************************************************
** CStockList: The stock list is responsible for recording scanned ArUco tags,
**             reading, writing, updating and deleting items from the record.
********************************************************************************/
class CStockList : public rclcpp::Node
{
public:
    // Create stock list which exports to the default file name "stock_<export-time>.txt"
    CStockList();
    
    // Create stock list which exports to the specified file name "<aFilePath>_<export-time>.txt"
    CStockList( std::string aFilePath );

    ~CStockList();

    // Retrieve the quantity of an item by its ArUco marker id.
    // @param aMarkerId The ArUco marker id of the item to query.
    // @return The item quantity.
    int GetQuantity( int aMarkerId );

    // Rename an item to a specified string by its ArUco marker id.
    // @param aItemId The ArUco marker id of the item to rename.
    // @param aName The string to rename the item to.
    void RenameItem( int aItemId, std::string aName );

    // Exports the current stock to "<mFileName>_<export-time>.txt"
    void WriteToDisk();

private:
    // Variables
    std::string mFileName;                      // The path which stock export to
    std::unordered_map< int, CItem > mItems;    // Map from ArUco id to item objects

    // ROS topic publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mPubStockUpdate;

    // ROS topic subscribers
    rclcpp::Subscription<std_msgs::msg::Integer>::SharedPtr mSubScannedMarkerId;

    // ROS subscriber callback updates stock record when new ArUco marker is scanned.
    void RecordStockCallback( const std_msgs::msg::Integer::SharedPtr msg );

    // Creates a new item instance with the specified parameters.
    // @param aMarkerId The ArUco marker id of the item.
    void CreateItem( int aMarkerId );

    // Retrieve pointer to item corresponding to specified ArUco marker id.
    // @param aMarkerId The ArUco marker id of the item.
    // @return Pointer to item object or NULL if no object with specified id.
    CItem* RetrieveItem( int aMarkerId );

    // Update item object corresponding to specified ArUco marker id.
    // @param aMarkerId The ArUco marker id of the item.
    // @param aQuantity The new quantity of the item.
    // @param aName The new name to assign to the item.
    void UpdateItem( int aMarkerId, int aQuantity, std::string aName );

    // Destruct item object corresponding to specified ArUco marker id.
    // @param aMarkerId The ArUco marker id of the item.
    void DeleteItem( int aMarkerId );

    // Initialises variables from stock export file.
    // @param aFilePath The path of the stock export file.
    void LoadFile( std::string aFilePath );
};

#endif