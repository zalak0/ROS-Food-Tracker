#ifndef ITEM
#define ITEM

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

    // Returns the ArUco marker id of the item
    // @return the ArUco marker id of the item or -1 if not found
    int GetId();

    // Updates the name of the item to the specified string
    // @param
    // - aName: the human-readable string to represent the item
    void SetName( std::string aName );

    // Returns the name of the item
    // @return the human-readable string representing the item
    std::string GetName();

    // Returns the quantity of the item
    // @return the quantity of the item
    int GetQuantity();

    // Update the quantity of the item
    // @param
    // - aQuantity: the quantity to update the item to
    void SetQuantity( int aQuantity );

    // Increase the quantity of the item by 1
    void IncrementQuantity();

private:
    int mMarkerId;      // The ArUco marker id of the item
    int mQuantity;      // The quantity of this item type
    std::string mName;  // The human-readable name of the item
};

#endif