#include "AndreiUtils/utilsBinarySerialization.hpp"
#include "Model_PreProsessing.h"
#include "Database.h"
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <unordered_map>
#include <chrono>

#include <pcl/point_types.h>




int main() {


    OuterMap generatedMap = DB::create_hashMap(0.01,0.001, "../YCB_ply/Selected_two");
    DB::to_serialize_hashMap(generatedMap, "Adel.bin");
    std::string filename = "Adel.bin";
    OuterMap deserialized_map = DB::to_deserialize_hashMap(filename);



    return 0;
}
