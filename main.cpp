
#include "Model_PreProsessing.h"
#include <iostream>
#include <Eigen/Core>
#include <algorithm>
#include <utility>
#include <vector>
#include <list>
#include<tuple>
#include <string>

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


typedef std::vector<Eigen::Matrix4f> RigidTransformation;


int main()
{

    auto Offline_HashTable = Compute_HashTable();

    /*
    // all variables here
    std::list <RigidTransformation> Hypothesis;
    float PS {0.9};
    float K {0.1};
    float C {0.6};
    long int m {0};
    long int n {0};
    long int N = -n log()




    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPLYFile<pcl::PointXYZ> ("Apple_in_Scene.ply", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }

    for (auto i : *cloud)
    {
        std::cout << i.x << " " << i.y << " " << i.z << " " << std::endl;
    }
    std::cout << cloud->size() << std::endl;


        float resolution = 0.005f;
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
        octree.setInputCloud (cloud);
        octree.addPointsFromInputCloud ();

        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledCloud(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::AlignedPointTVector voxel_centers; // voxel_centers is the new cloud after downsampling
        octree.getOccupiedVoxelCenters(voxel_centers);

        downsampledCloud->points.resize(voxel_centers.size());
        std::copy(voxel_centers.begin(), voxel_centers.end(), downsampledCloud->points.begin());

        std::cout << "num of points after downsampling " << voxel_centers.size() << std::endl;
*/



    return 0;
}

