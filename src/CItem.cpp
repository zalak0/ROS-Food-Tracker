#include "turtlebot3_gazebo/CItem.hpp"


CItem::CItem( int aMarkerId )
: mMarkerId( aMarkerId ),
  mQuantity( 0 ),
  mName( "Unnamed Item" )
{}


CItem::CItem( int aMarkerId, int aQuantity, std::string aName )
: mMarkerId( aMarkerId ),
  mQuantity( aQuantity ),
  mName( aName )
{}


int CItem::GetId()
{
    return mMarkerId;
}


void CItem::SetName( std::string aName )
{
    mName = aName;
}


std::string CItem::GetName()
{
    return mName;
}


int CItem::GetQuantity()
{
    return mQuantity;
}


void CItem::SetQuantity( int aQuantity )
{
    mQuantity = aQuantity;
}


void CItem::IncrementQuantity()
{
    mQuantity++;
}
