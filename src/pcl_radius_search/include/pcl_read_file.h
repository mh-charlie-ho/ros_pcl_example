#ifndef PCL_READ_FILE_H
#define PCL_READ_FILE_H

#include <pcl/io/pcd_io.h>
#include <string>

class PointCloudReader
{
public:
    PointCloudReader(
        std::string filename,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    PointCloudReader(
        std::string filename,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudRGB);
    
    void Read(std::string filename);
    void ReadRGB(std::string filename);

private:
    pcl::PCDReader mReader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mCloudRGB;
};

#endif