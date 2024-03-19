#include <ros/ros.h>
#include <pcl/filters/extract_indices.h>
// #include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <string>

#include "radius_search_traversal_charlie.h"
#include "ellipsoid_clustering.cpp"
#include "../include/util.h"

static void ConvertROSMsgToPCLPoint(
    const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &sensor_cloud_ptr)
{
    float height = in_sensor_cloud->height;
    float width = in_sensor_cloud->width;
    float point_step = in_sensor_cloud->point_step;
    float row_step = in_sensor_cloud->row_step;
    float field_size = in_sensor_cloud->fields.size();

    std::cout << "data width: " << width << std::endl;

    for (int i = 0; i < width; i++)
    {
        pcl::PointXYZ p;
        std::memcpy(&p.x, &in_sensor_cloud->data[point_step * i], 4);
        std::memcpy(&p.y, &in_sensor_cloud->data[point_step * i + 4], 4);
        std::memcpy(&p.z, &in_sensor_cloud->data[point_step * i + 8], 4);

        sensor_cloud_ptr->points.push_back(p);
    }
}

void ReceivePointCloud(
    const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::fromROSMsg(*in_sensor_cloud, *sensor_cloud_ptr);
    ConvertROSMsgToPCLPoint(in_sensor_cloud, sensor_cloud_ptr);

    std::cout << "kdtree radius search" << std::endl;
    std::cout << "total points: " << sensor_cloud_ptr->size() << std::endl;

    // pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // std::vector<int> idx = {0};
    // inliers->indices = idx;

    // pcl::ExtractIndices<pcl::PointXYZ> extract;
    // extract.setInputCloud(sensor_cloud_ptr);
    // extract.setIndices(inliers);
    // extract.setNegative(false);
    // extract.filter(*cloud_plane);

    // std::cout << "total points: " << cloud_plane->size() << std::endl;

    // RadiusSearch R;

    // R.OctreeRadiusSearch(cloud_plane);
    // R.KdtreeRadiusSearch(cloud_plane);

    // R.GPUOctreeRadiusSearch(cloud_plane);

    ellipsoidalClustering ER(2.0, 2.0, 7.5);

    util::Timer clock("clock");
    ER.ellipsoidalClustering_main(sensor_cloud_ptr);
    clock.stop();
}

/// ============================================================================
int main(int argc, char **argv)
{
    std::cout << "Start" << std::endl;

    ros::init(argc, argv, "radius_search");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    std::string topic_name;
    nhp.param<std::string>("topic_name", topic_name, "lidar_points");
    std::cout << topic_name << std::endl;

    ros::Subscriber sub = nh.subscribe<const sensor_msgs::PointCloud2ConstPtr &>(topic_name, 1, &ReceivePointCloud);
    ros::spin();

    return 0;
}