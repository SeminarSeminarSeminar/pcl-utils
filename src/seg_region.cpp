#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>

int main(int argc, char **argv)
{
    if (argc < 3) {
        std::cout << "Usage: " << "./seg_region input.pcd output.pcd [NumberOfNeighbours(default=300)]" << std::endl;
        return -1;
    }

    int neighbors = argc > 3 ? atoi(argv[3]) : 300;

    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *in_cloud);

    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(in_cloud);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normal_cloud);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize(50);
    reg.setMaxClusterSize(100000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(neighbors);
    reg.setInputCloud(in_cloud);
    reg.setInputNormals(normal_cloud);
    reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(1.0);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    pcl::io::savePCDFile<pcl::PointXYZRGB>(argv[2], *colored_cloud);
    // Red points indicate regions that were not segmented. Must be removed manually. (Color code: 4294901760)

    // TODO: Normal vector of a horizontal surface. Must be set manually
    float x = -0.121292, y = -0.986574, z = 0.109363;

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr region_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (int i = 0; i < clusters.size(); i++) {
        pcl::PointIndices::Ptr inliners(new pcl::PointIndices(clusters[i]));
        extract.setInputCloud(in_cloud);
        extract.setIndices(inliners);
        extract.setNegative(false);
        extract.filter(*region_cloud);

        pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
        pcl::PointIndices::Ptr inliners2(new pcl::PointIndices());
        pcl::ExtractIndices<pcl::PointXYZ> extract2;
        pcl::SACSegmentation<pcl::PointXYZ> filter_seg;

        filter_seg.setOptimizeCoefficients(true);
        filter_seg.setModelType(pcl::SACMODEL_PLANE);
        filter_seg.setMethodType(pcl::SAC_RANSAC);
        filter_seg.setMaxIterations(1000);
        filter_seg.setDistanceThreshold(0.1);
        filter_seg.setInputCloud(region_cloud);
        filter_seg.segment(*inliners2, *coeff);

        float product = coeff->values[0] * x + coeff->values[1] * y + coeff->values[2] * z;

        if (product < 0.707 && product > -0.707) {
            // Vertical surface
            std::stringstream ss;
            ss << "region_vertical_" << i << ".pcd";
            pcl::io::savePCDFile<pcl::PointXYZ>(ss.str(), *region_cloud);
        } else {
            // Horizontal surface
            std::stringstream ss;
            ss << "region_horizontal_" << i << ".pcd";
            pcl::io::savePCDFile<pcl::PointXYZ>(ss.str(), *region_cloud);
        }
    }

    return 0;
}