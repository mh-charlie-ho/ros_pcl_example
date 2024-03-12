#include "radius_search_traversal_charlie.h"

int main()
{
    RadiusSearch R;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sensor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    R.OctreeRadiusSearch(sensor_cloud_ptr);
    
    return 0;
}