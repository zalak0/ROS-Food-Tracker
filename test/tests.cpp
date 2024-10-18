#include "turtlebot3_gazebo/CStockList.hpp"
#include "turtlebot3_gazebo/CItem.hpp"

#include <iostream>


int main( int argc, char *argv[] )
{ 
  const std::string filePath = argv[1];
  
  CStockList list = CStockList();
  list.WriteToDisk();

  return 0;
}