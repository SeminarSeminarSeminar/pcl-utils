#include <iostream>
#include <stdlib.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/radius_outlier_removal.h>

int apply_radius_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, double radius, int min_pts) 
{
    std::cout << "Filter settings: Radius Outlier Removal (RadiusSearch = " << radius << ", MinNeighborsInRadius = " << min_pts << ")" << std::endl;

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter;
    filter.setInputCloud(in);
    filter.setRadiusSearch(radius);
    filter.setMinNeighborsInRadius(min_pts);
    filter.filter(*out);

    return 0;
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        std::cout << "Usage: " << "./filter_radius input_file output_file [RadiusSearch(default=0.01)] [MinNeighborsInRadius(default=10)]" << std::endl;
        return -1;
    }

    double radius = argc > 3 ? atof(argv[3]) : 0.01;
    int min_pts = argc > 4 ? atoi(argv[4]) : 10;

    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *in_cloud);
    std::cout << "Input cloud: " << in_cloud->size() << " points" << std::endl;

    if (apply_radius_filter(in_cloud, out_cloud, radius, min_pts) < 0) {
        std::cout << "Error" << std::endl;
        return -1;
    }

    pcl::io::savePCDFile<pcl::PointXYZ>(argv[2], *out_cloud);
    std::cout << "Output cloud: " << out_cloud->size() << " points" << std::endl;

    return 0;
}