#ifndef STOCK_LIST
#define STOCK_LIST

#include <string>
#include <unordered_map>


/********************************************************************************
** CItem: Items represent things for sale in the store, represented by ArUco
**        markers.
********************************************************************************/
class CItem
{
public:
    CItem( int aMarkerId );
    CItem( int aMarkerId, int aQuantity, std::string aName );

    // GetId: Returns the ArUco marker id of the item
    // Returns: the ArUco marker id of the item or -1 if not found
    int GetId();

    // SetName: Updates the name of the item to the specified string
    // Parameters:
    // - aName: the human-readable string to represent the item
    void SetName( std::string aName );

    // GetName: Returns the name of the item
    // Returns: the human-readable string representing the item
    std::string GetName();

    // GetQuantity: Returns the quantity of the item
    // Returns: the quantity of the item
    int GetQuantity();

    // SetQuantity: Update the quantity of the item
    // Parameters:
    // - aQuantity: the quantity to update the item to
    void SetQuantity( int aQuantity );

    // IncrementQuantity: Increase the quantity of the item by 1
    void IncrementQuantity();

private:
    int mMarkerId;      // The ArUco marker id of the item
    int mQuantity;      // The quantity of this item type
    std::string mName;  // The human-readable name of the item
};

#endif