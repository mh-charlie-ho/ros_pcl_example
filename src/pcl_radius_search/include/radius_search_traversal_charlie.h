#ifndef RADIUS_SEARCH_TRAVERSAL_CHARLIE_H
#define RADIUS_SEARCH_TRAVERSAL_CHARLIE_H

#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <iostream>
#include <vector>
#include <ctime>
#include <pcl/io/pcd_io.h>

class RadiusSearch
{
public:
    void GPUOctreeRadiusSearch(pcl::PointCloud<pcl::PointXYZ>::Ptr &);
    void OctreeRadiusSearch(pcl::PointCloud<pcl::PointXYZ>::Ptr &);
    void KdtreeRadiusSearch(pcl::PointCloud<pcl::PointXYZ>::Ptr &);
};

#endif