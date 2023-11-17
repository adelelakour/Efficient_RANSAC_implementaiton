//
// Created by adelelakour on 08.11.23.
//

#ifndef MY_HEADER_H
#define MY_HEADER_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <utility>
#include <vector>
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


typedef std::pair<pcl::PointXYZLNormal, pcl::PointXYZLNormal> PAIR;
typedef std::tuple <PAIR, std::string> Element_in_Cell;
typedef std::vector<Element_in_Cell> WholeCell;

typedef std::unordered_map<std::string, std::vector<PAIR>> Cell;
typedef std::unordered_map<std::string, Cell> HashMap;

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

double Angle_Two_Vectors(Eigen::Vector3f Vector_A, Eigen::Vector3f Vector_B);
double Euclidean_Distance_two_Vectors(Eigen::Vector3f A, Eigen::Vector3f B);
Eigen::Vector3f U_sub_V(Eigen::Vector3f A, Eigen::Vector3f B);
Eigen::Vector3f V_sub_U(Eigen::Vector3f A, Eigen::Vector3f B);

HashMap Compute_HashTable(float radius = 0.01, double pointSphereRelativeTolerance = 0.01);

#endif // MY_HEADER_H


