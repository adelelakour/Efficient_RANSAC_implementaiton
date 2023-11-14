cmake_minimum_required(VERSION 3.25.2)
project(Effi_RANSAC_MA)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.13 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(Effi_RANSAC_MA main.cpp
        Model_PreProsessing.cpp
        Model_PreProsessing.h)
target_link_libraries(Effi_RANSAC_MA ${PCL_LIBRARIES})
