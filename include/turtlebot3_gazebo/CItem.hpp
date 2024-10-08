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
    // Getters and setters for each member variable, probably overload for elegance

private:
    int mMmarkerId;      // The ArUco marker id of the item
    int mQuantity;       // The quantity of this item type
    std::string mName;   // The human-readable name of the item
};

#endif