cmake_minimum_required(VERSION 3.16)
project(Effi_RANSAC_MA)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.13 REQUIRED)
#find_package(AndreiUtils REQUIRED core)

include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(ObjectModel
        Database.cpp
        Model_PreProsessing.cpp
        )
target_link_libraries(ObjectModel ${AndreiUtils_LIBRARY} ${PCL_LIBRARIES})

add_executable(Effi_RANSAC_MA main.cpp)
target_link_libraries(Effi_RANSAC_MA ObjectModel)

add_executable(TestBla test.cpp)
target_link_libraries(TestBla ObjectModel)
