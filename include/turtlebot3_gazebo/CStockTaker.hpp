#ifndef STOCK_TAKER
#define STOCK_TAKER

#include "std_msgs/msg/integer.hpp"


/********************************************************************************
** CStockTaker: The stock taker class is responsible for listening for scanned
**              ArUco marker ids and adding them to the stock list when heard.
********************************************************************************/
class CStockTaker : public rclcpp::Node
{
public:
    CStockTaker();
    ~CStockTaker();

private:
    // ROS topic subscribers
    rclcpp::Subscription<std_msgs::msg::Integer>::SharedPtr mSubScannedMarkerId;

    // RecordStockCallback: Interacts with CStockList to add the new stock item.
    void RecordStockCallback();
};

#endif
