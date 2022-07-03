#include <iostream>
#include <stdlib.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

int apply_stat_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, int meank, double stddevmulthresh) 
{
    std::cout << "Filter settings: Statistical Outlier Removal (MeanK = " << meank << ", StddevMulThresh = " << stddevmulthresh << ")" << std::endl;

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;
    filter.setInputCloud(in);
    filter.setMeanK(meank);
    filter.setStddevMulThresh(stddevmulthresh);
    filter.filter(*out);

    return 0;
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        std::cout << "Usage: " << "./filter_stat input_file output_file [MeanK(default=50)] [StddevMulThresh(default=1.0)]" << std::endl;
        return -1;
    }

    int meank = argc > 3 ? atoi(argv[3]) : 50;
    double threshold = argc > 4 ? atof(argv[4]) : 1.0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *in_cloud);
    std::cout << "Input cloud: " << in_cloud->size() << " points" << std::endl;

    if (apply_stat_filter(in_cloud, out_cloud, meank, threshold) < 0) {
        std::cout << "Error" << std::endl;
        return -1;
    }

    pcl::io::savePCDFile<pcl::PointXYZ>(argv[2], *out_cloud);
    std::cout << "Output cloud: " << out_cloud->size() << " points" << std::endl;

    return 0;
}