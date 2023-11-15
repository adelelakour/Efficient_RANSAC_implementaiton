#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <utility>
#include <vector>
#include <set>
#include <list>
#include<tuple>
#include <string>

#include <filesystem>
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
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/registration/correspondence_estimation.h>

namespace fs = std::filesystem;
namespace std {
    template<>
    struct hash<Eigen::Vector3f> {
        std::size_t operator()(const Eigen::Vector3f &vec) const {
            // Combine the hashes of individual components of the Eigen::Vector3f
            size_t hash = std::hash<double>{}(vec.x());
            hash ^= std::hash<double>{}(vec.y()) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
            hash ^= std::hash<double>{}(vec.z()) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
            return hash;
        }
    };
}



Eigen::Vector3f Hash_key = {0,0,0};
Eigen::Vector3f An;
Eigen::Vector3f Bn;
Eigen::Vector3f A_B;
Eigen::Vector3f B_A;
double dotProduct;
double magnitudeA;
double magnitudeB;
double angleRadians;
double angleDegrees;




typedef std::pair<pcl::PointXYZLNormal, pcl::PointXYZLNormal> PAIR;
typedef std::tuple <std::vector<PAIR>, std::set<std::string>> Cell;


double Angle_Two_Vectors(Eigen::Vector3f Vector_A, Eigen::Vector3f Vector_B)
{

    dotProduct = Vector_A.dot(Vector_B);
    magnitudeA = Vector_A.norm();
    magnitudeB = Vector_B.norm();

    // Compute the angle in radians
    angleRadians = std::acos(dotProduct / (magnitudeA * magnitudeB));

    // Convert the angle to degrees
    angleDegrees = angleRadians * 180.0 / M_PI;

    return angleDegrees;
}

inline double Euclidean_Distance_two_Vectors(Eigen::Vector3f A, Eigen::Vector3f B)
{
    double Distance = std::sqrt(
            std::pow(B.x() - A.x(), 2) +
            std::pow(B.y() - A.y(), 2) +
            std::pow(B.z() - A.z(), 2)
    );
    return Distance;
}

inline Eigen::Vector3f U_sub_V(Eigen::Vector3f A, Eigen::Vector3f B)
{
    Eigen::Vector3f U_sub_V;
    A_B.x() = A.x()-B.x();
    A_B.y() = A.y()-B.y();
    A_B.z() = A.z()-B.z();
    return U_sub_V;
}

inline Eigen::Vector3f V_sub_U (Eigen::Vector3f A, Eigen::Vector3f B)
{
    Eigen::Vector3f V_sub_U;
    B_A.x() = B.x()-A.x();
    B_A.y() = B.y()-A.y();
    B_A.z() = B.z()-A.z();
    return V_sub_U;
}



std::unordered_map<Eigen::Vector3f, Cell> Compute_HashTable()
{

    std::unordered_map<Eigen::Vector3f, Cell> myHashTable;       // Cell is a tuple of (pair) and (model Name)
    myHashTable.clear();

    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr Model_Cloud(new pcl::PointCloud<pcl::PointXYZLNormal>);


    pcl::PointXYZLNormal U_XYZNorm;
    pcl::PointXYZLNormal V_XYZNorm;
    Eigen::Vector3f U_n = Eigen::Vector3f::Zero();
    Eigen::Vector3f V_n = Eigen::Vector3f::Zero();
    Eigen::Vector3f U_p = Eigen::Vector3f::Zero();
    Eigen::Vector3f V_p = Eigen::Vector3f::Zero();

    PAIR Detected_pairs;
    float radius {0.01};
    Cell OneCell;

    std::vector<int> point_indices;
    std::vector<float> point_distances;


    std::string directory_path = "../YCB_ply/Selected_two";
    std::string extension = ".ply";
    std::string Model_Name;
    std::string file_path;


    for (const auto &entry: fs::directory_iterator(directory_path)) {
        if (entry.is_regular_file() && entry.path().extension() == extension) {
            file_path = entry.path().string();
            Model_Name = entry.path().filename().stem();
        }


        pcl::io::loadPLYFile(file_path, *Model_Cloud);

        std::cout << "size before voxelgrid : " << Model_Cloud->size() << std:: endl;

        pcl::VoxelGrid<pcl::PointXYZLNormal> sor;
        sor.setInputCloud (Model_Cloud);
        sor.setLeafSize (0.003f, 0.003f, 0.003f);
        sor.filter (*Model_Cloud);
        std::cout << "size after voxelgrid  : " << Model_Cloud->size() << std:: endl;

        std::cout << "A new Model is loaded" << std::endl;

        pcl::KdTreeFLANN<pcl::PointXYZLNormal> kdtree_of_Model_Cloud;
        kdtree_of_Model_Cloud.setInputCloud(Model_Cloud);


        for (size_t i = 1; i < Model_Cloud->size(); ++i) {
            auto U_XYZNorm = Model_Cloud->at(i);

            U_p = Eigen::Vector3f(U_XYZNorm.x, U_XYZNorm.y, U_XYZNorm.z);
            U_n = Eigen::Vector3f(U_XYZNorm.normal_x, U_XYZNorm.normal_y, U_XYZNorm.normal_z);

            kdtree_of_Model_Cloud.radiusSearch(U_XYZNorm, radius, point_indices, point_distances);
            //std::cout << "number of points within this sphere is : " << point_indices.size() << std::endl;

            for (std::size_t i = 0; i < point_indices.size(); ++i) {
                V_XYZNorm = Model_Cloud->at(point_indices[i]);

                if (euclideanDistance (U_XYZNorm, V_XYZNorm) > radius - (radius * 0.01) && euclideanDistance (U_XYZNorm, V_XYZNorm) < radius + (radius * 0.01)) { // Tolerance for considering a point on the perimeter
                    //std::cout << "I found a Pair in " << Model_Name << std::endl;
                    V_p = Eigen::Vector3f(V_XYZNorm.x,
                                          V_XYZNorm.y,
                                          V_XYZNorm.z);

                    V_n = Eigen::Vector3f(V_XYZNorm.normal_x,
                                          V_XYZNorm.normal_y,
                                          V_XYZNorm.normal_z);

                    Hash_key[0] = Angle_Two_Vectors(U_n, V_n);

                    auto Pu_Pv = U_sub_V(U_p, V_p);
                    auto Pv_Pu = V_sub_U(U_p, V_p);

                    Hash_key[1] = Angle_Two_Vectors(U_n, Pv_Pu);
                    Hash_key[2] = Angle_Two_Vectors(V_n, Pu_Pv);

                    Detected_pairs.first = U_XYZNorm;
                    Detected_pairs.second = V_XYZNorm;

                    if (myHashTable.find(Hash_key) != myHashTable.end())
                    {
                        std::get<0> (myHashTable[Hash_key]).push_back(Detected_pairs);
                        std::get<1> (myHashTable[Hash_key]).insert(Model_Name);
                        std::cout << "A new entry is added to existing key:" << std::endl;
                    }
                    else
                    {
                        Cell newData;
                        std::get<0>(newData).push_back(Detected_pairs);
                        std::get<1>(newData).insert(Model_Name);
                        myHashTable[Hash_key] = newData;
                        std::cout << "A new key is added :" << std::endl;
                    }

                    //break;
                }
            }
        }
    }

    std::cout << "Size of myHashTable is :" << myHashTable.size() << std::endl;


    for (auto entity : myHashTable)
    {
        auto Cell = entity.second;
        if (std::get<0>(Cell).size()  > 2)
        {
            std::cout << "I found big cell" << std::endl;
        }
    }
    std::cout << "NNNNNNN" << std::endl;


    return myHashTable;
}


