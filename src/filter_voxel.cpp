#include <iostream>
#include <stdlib.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

int apply_voxel_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, double lx, double ly, double lz) 
{
    std::cout << "Filter settings: Voxel Grid (LeafSize: lx = " << lx << ", ly = " << ly << ", lz = " << lz << ")" << std::endl;

    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(in);
    filter.setLeafSize(lx, ly, lz);
    filter.filter(*out);

    return 0;
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        std::cout << "Usage: " << "./filter_voxel input_file output_file [LeafSize(default=0.05)]" << std::endl;
        return -1;
    }

    double leaf_size = argc > 3 ? atof(argv[3]) : 0.05;

    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *in_cloud);
    std::cout << "Input cloud: " << in_cloud->size() << " points" << std::endl;

    if (apply_voxel_filter(in_cloud, out_cloud, leaf_size, leaf_size, leaf_size) < 0) {
        std::cout << "Error" << std::endl;
        return -1;
    }

    pcl::io::savePCDFile<pcl::PointXYZ>(argv[2], *out_cloud);
    std::cout << "Output cloud: " << out_cloud->size() << " points" << std::endl;

    return 0;
}