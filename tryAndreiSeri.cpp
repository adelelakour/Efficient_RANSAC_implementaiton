#include "AndreiUtils/utilsBinarySerialization.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <unordered_map>
//std::unordered_map <std::string, std::unordered_map <std::string, vector<pair<pcl::PointXYZLNormal, pcl::PointXYZLNormal>>>
#include <pcl/common/distances.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/octree/octree_search.h>



struct struct_of_PCL_Point {
    float x, y, z;
    int label;
    float normal_x, normal_y, normal_z;
};


struct_of_PCL_Point convertToPointXYZLNormal(const pcl::PointXYZLNormal& pclPoint) {
    struct_of_PCL_Point myPoint;
    myPoint.x = pclPoint.x;
    myPoint.y = pclPoint.y;
    myPoint.z = pclPoint.z;
    myPoint.label = pclPoint.label;
    myPoint.normal_x = pclPoint.normal_x;
    myPoint.normal_y = pclPoint.normal_y;
    myPoint.normal_z = pclPoint.normal_z;
    return myPoint;
}



pcl::PointXYZLNormal convertToPCLPointXYZLNormal(const struct_of_PCL_Point& myPoint) {
    pcl::PointXYZLNormal pclPoint;
    pclPoint.x = myPoint.x;
    pclPoint.y = myPoint.y;
    pclPoint.z = myPoint.z;
    pclPoint.label = myPoint.label;
    pclPoint.normal_x = myPoint.normal_x;
    pclPoint.normal_y = myPoint.normal_y;
    pclPoint.normal_z = myPoint.normal_z;
    return pclPoint;
}


struct test
{
    int x {0};
    int y {20};
    std::string label;
};

namespace AndreiUtils{
    void serialize(std::ofstream &out, std::string const &data) {
        serialize(out, data.length());
        for (const char &character : data) {
            out.write(&character, sizeof(character));
        }
    }


    // Here Is deserialize String
    std::string deserialize(std::ifstream &in) {
        // Read the string length
        size_t length;
        AndreiUtils::deserialize(in, length);

        // Allocate memory for the string
        char *buffer = new char[length + 1]; // +1 for the null terminator

        // Read the string characters
        for (size_t i = 0; i < length; i++) {
            in.read(&buffer[i], sizeof(char));
        }
        buffer[length] = '\0'; // Add the null terminator

        // Create and return the string
        std::string str(buffer);
        delete[] buffer;

        return str;
    }


/*    void serialize(std::ofstream OUT, const test data)
    {
        serialize(OUT, data.x);
        serialize(OUT, data.y);
        serialize(OUT, data.label);
        char buffer[sizeof (test)];
        std::memcpy( buffer, data.x, sizeof(test))
        OUT.write()
    }*/

}





int main() {


    std::string Name = "AdelElakour";
    std::ofstream outFile("serialized_string.bin", std::ios::binary);
    AndreiUtils::serialize(outFile, Name);



    /*
    test s {2,4,"Orange"};

    std::ofstream outFile("serialized_struct.bin", std::ios::binary);
    AndreiUtils::serialize(outFile, s);
    outFile.close();


    /*
    std::ifstream inFile("serialized_vector.bin", std::ios::binary);
    int deserializedVector [5];
    int arraySize = sizeof(deserializedVector) / sizeof(deserializedVector[0]);

    AndreiUtils::deserialize(inFile, deserializedVector, arraySize);

    inFile.close();
    for (const auto &element : deserializedVector) {
        std::cout << element << " ";
    }
    std::cout << std::endl;

    /*
     *
    template<typename T>
    void deserialize(std::ifstream &in, T *data, size_t nrElements) {
        for (size_t i = 0; i < nrElements; i++) {
            deserialize(in, data[i]);
        }
    }
    */





    return 0;
}