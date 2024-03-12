#include "pcl_read_file.h"

#include <string>

PointCloudReader::PointCloudReader(
    std::string filename,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    : mCloud(cloud)
{
    Read(filename);
}

PointCloudReader::PointCloudReader(
    std::string filename,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudRGB)
    : mCloudRGB(cloudRGB)
{
    ReadRGB(filename);
}

void PointCloudReader::Read(std::string filename)
{
    mReader.read(filename, *mCloud);
}

void PointCloudReader::ReadRGB(std::string filename)
{
    mReader.read(filename, *mCloudRGB);
}