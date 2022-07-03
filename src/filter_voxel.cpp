#include <iostream>
#include <stdlib.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

int apply_voxel_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, pcl::PointCloud<pcl::PointXYZ>::Ptr grid, double lx, double ly, double lz) 
{
    std::cout << "Filter settings: Voxel Grid (LeafSize: lx = " << lx << ", ly = " << ly << ", lz = " << lz << ")" << std::endl;

    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(in);
    filter.setLeafSize(lx, ly, lz);
    filter.filter(*out);

    // Get grid coordinates
    grid->width = out->size();
    grid->height = 1;
    grid->is_dense = false;
    grid->points.resize(grid->width * grid->height);

    for (int i = 0; i < grid->points.size(); i++) {
        Eigen::Vector3i grid_vector = filter.getGridCoordinates(out->points[i].x, out->points[i].y, out->points[i].z);
        grid->points[i].x = grid_vector[0];
        grid->points[i].y = grid_vector[1];
        grid->points[i].z = grid_vector[2];
    }

    return 0;
}

int main(int argc, char **argv)
{
    if (argc < 4) {
        std::cout << "Usage: " << "./filter_voxel input_pcd output_pcd output_grid [LeafSize(default=0.05)]" << std::endl;
        return -1;
    }

    double leaf_size = argc > 4 ? atof(argv[4]) : 0.05;

    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr grid_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *in_cloud);
    std::cout << "Input cloud: " << in_cloud->size() << " points" << std::endl;

    if (apply_voxel_filter(in_cloud, out_cloud, grid_cloud, leaf_size, leaf_size, leaf_size) < 0) {
        std::cout << "Error" << std::endl;
        return -1;
    }

    pcl::io::savePCDFile<pcl::PointXYZ>(argv[2], *out_cloud);
    std::cout << "Output cloud: " << out_cloud->size() << " points" << std::endl;

    pcl::io::savePCDFile<pcl::PointXYZ>(argv[3], *grid_cloud);
    std::cout << "Output grid: " << grid_cloud->size() << " points" << std::endl;

    return 0;
}