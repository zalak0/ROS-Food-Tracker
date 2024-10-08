#ifndef STOCK_LIST
#define STOCK_LIST

#include <string>
#include <unordered_map>


//---Forward Declarations------------------------------------------------------
class CItem;


/********************************************************************************
** CStockList: The stock list is responsible for reading, writing, updating and
**             deleting data from a text stock file.
********************************************************************************/
class CStockList
{
public:
    // Creates a new stock file with the default name "stock.txt"
    CStockList();
    
    // Loads stock from a specified file if it exists, otherwise creates a stock
    // file with the specified file name and path.
    CStockList( std::string aFilePath );

    ~CStockList();

    // Add line for new item if 
    void CreateItem( int aMarkerId );

    // Lookup item by id
    void RetrieveItem( int aMarkerId );

    // Update item by id with supplied details (aNewInfo must match aMarkerId)
    void UpdateItem( int aMarkerId, CItem aNewInfo );

    // Delete item by id
    void DeleteItem( int aMarkerId );

private:
    // Variables
    std::string mFileName;
    std::unordered_map< int, CItem > mItems;

    // CRUD operations should be separated into private functions where appropriate

    // Loads items from specified file into member variable
    void LoadFile( std::string aFilePath ); // Nice to have (no rush to implement)
};

#endif