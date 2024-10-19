#include "turtlebot3_gazebo/CStockList.hpp"
#include "turtlebot3_gazebo/CItem.hpp"

#include <iostream>


int main( int argc, char *argv[] )
{ 
  CStockList list = CStockList();
  list.TestRecordStockCallback( 15 );
  list.TestRecordStockCallback( 15 );
  list.RenameItem( 15, "Banana Bread" );
  list.TestRecordStockCallback( 5 );
  list.RenameItem( 5, "Ice Cream" );
  list.TestRecordStockCallback( 5 );
  list.TestRecordStockCallback( 5 );
  list.WriteToDisk();

  return 0;
}