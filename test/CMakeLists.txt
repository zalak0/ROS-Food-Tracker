cmake_minimum_required(VERSION 3.10)
project(LogicSimulator)
set(CMAKE_CXX_FLAGS "-Wall") # Show warnings

set( CMAKE_RUNTIME_OUTPUT_DIRECTORY # Destination for generated executable
    ${CMAKE_SOURCE_DIR}/bin
)

set ( CMAKE_CXX_STANDARD 11 )

include_directories( # Header files
    "../include"
)

add_executable( stock_tests
                tests.cpp
                ../src/CItem.cpp
                ../src/CStockList.cpp
)