#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

// Usage: ./concat [output file] [input file 1] [input file 2] ...
int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (int i = 2; i < argc; i++) {
        pcl::io::loadPCDFile<pcl::PointXYZ>(argv[i], *in_cloud);
        *out_cloud += *in_cloud;
    }

    pcl::io::savePCDFile<pcl::PointXYZ>(argv[1], *out_cloud);

    return 0;
}