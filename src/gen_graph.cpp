#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <algorithm>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/common/distances.h> 
#include <typeinfo>

// Definitions of Node Type
#define INVALID             -1
#define FLOOR               0
#define CIELING             1 
#define WALL                2
#define FLOOR_CANDIDATE     3
#define CEILING_CANDIDATE   4
#define WALL_CANDIDATE      5
// Definitions of Edge Type
#define INVALID_EDGE             0
#define WALL_WALL           1
#define FLOOR_FLOOR         2
#define CEILING_CEILING     3
#define FLOOR_WALL          4
#define CEILING_WALL        5

#define MERGE_DIST_THRESH     (0.1)
#define MERGE_ANGLE_THRESH     (20)
#define ADJ_THRESHOLD       (1.5)

using std::vector;
using std::cout;

float CalcMinDistance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloudA, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloudB, float target = 0){
    float min = 100;
    for(auto a_p : pCloudA->points){
        for(auto b_p : pCloudB->points){
            double distance = pcl::euclideanDistance(a_p, b_p);
            if(distance < target) return distance;
            if(min > distance) min = distance;
            
        }
    }
	return min;
}



class Segment{
public:
    Segment(): mpCloud(new pcl::PointCloud<pcl::PointXYZRGB>){

    };
    Segment(pcl::PointCloud<pcl::PointXYZRGB> cloud, Eigen::Vector4f planeCoef, float curvature, pcl::PointXYZ centroid, int disabled):
        mpCloud(new pcl::PointCloud<pcl::PointXYZRGB>),mPlaneCoef(planeCoef), mCurvature(curvature), mCentroid(centroid){
        *mpCloud = cloud;
        mDegree = pcl::getAngle3D(planeCoef.cwiseAbs(), Eigen::Vector4f(0, 1, 0, 0), true);
        mDisabled = disabled;
    };
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mpCloud;
    Eigen::Vector4f mPlaneCoef;
    float mCurvature = 0;
    float mDegree = 0;
    pcl::PointXYZ mCentroid;
    int mType = INVALID;    
	int mDisabled = 0;
};


bool IsMerged(Segment s){
    return s.mDisabled==1;
}

class AdjacencyGraph{
public:
    AdjacencyGraph(){

    };
    void MergeOverlappingSegments(){
        // TODO: Merge overlaping point cloud
        cout << "Node num before merged :" << mvSegments.size() <<"\n";
        for(auto i = 0; i < mvSegments.size(); i++){
            for(auto j = i+1; j < mvSegments.size(); j++){
                if (mvSegments[i].mDisabled == 1) continue;
                float degree = pcl::getAngle3D(mvSegments[i].mPlaneCoef, mvSegments[j].mPlaneCoef, true);
                if(pcl::getAngle3D(mvSegments[i].mPlaneCoef, mvSegments[j].mPlaneCoef, true) > MERGE_ANGLE_THRESH) continue;                
                double distance = CalcMinDistance(mvSegments[i].mpCloud, mvSegments[j].mpCloud, MERGE_DIST_THRESH);
                
                if(distance < MERGE_DIST_THRESH){
                    *mvSegments[i].mpCloud += *mvSegments[j].mpCloud;
                    for (auto pt: mvSegments[j].mpCloud->points){
                        pt.rgb = mvSegments[i].mpCloud->points[0].rgb;
                    }
                    mvSegments[j].mDisabled = 1;
                    //pcl::io::savePCDFile<pcl::PointXYZRGB>("merged_"+std::to_string(i)+"_"+std::to_string(j), *mvSegments[i].mpCloud); 
                }
            }
        }
        mvSegments.erase(std::remove_if(mvSegments.begin(), mvSegments.end(), IsMerged), mvSegments.end());
        cout << "Node num before merged :" << mvSegments.size() <<"\n";
        mNodenum = mvSegments.size();
    }
    void ConstructGraph(){        
        mNodenum = mvSegments.size();
        
        float y = 0;
        // Set Segment Type
        for(auto &s: mvSegments){
            if(s.mDegree > 45){
                s.mType = WALL_CANDIDATE;
                y += s.mCentroid.y;
            }
        }
        y = y/mvSegments.size();
        Eigen::Vector4f newNormal;
        for(auto &s: mvSegments){
            if(s.mDegree < 45 && s.mCentroid.y < y){
                s.mType = FLOOR_CANDIDATE;
                newNormal = s.mPlaneCoef;
            } else if(s.mDegree < 45 && s.mCentroid.y > y){
                s.mType = CEILING_CANDIDATE;
            }
        }
        
        // Correct Degree with new Normal vector
        for(auto &s: mvSegments){
            s.mDegree = pcl::getAngle3D(s.mPlaneCoef, newNormal, true);
        }

        // Connect Edge
        mpGraph = new int[mNodenum * mNodenum];
        std::fill(&mpGraph[0], &mpGraph[mNodenum*mNodenum], INVALID_EDGE);
        for(auto i = 0; i < mNodenum; i++){
            for(auto j = i+1; j < mNodenum; j++){
                if(pcl::getAngle3D(mvSegments[i].mPlaneCoef.cwiseAbs(), mvSegments[j].mPlaneCoef, true) < 45) continue;
                double distance = pcl::euclideanDistance(mvSegments[i].mCentroid, mvSegments[j].mCentroid);
                //double distance = CalcMinDistance(mvSegments[i].mpCloud, mvSegments[j].mpCloud, ADJ_THRESHOLD);
                cout << mvSegments[i].mpCloud->size() << " " << mvSegments[j].mpCloud->size() << " " << distance << "\n";
                if(distance < ADJ_THRESHOLD){
                    int type1 = mvSegments[i].mType;
                    int type2 = mvSegments[j].mType;
                    int type = INVALID_EDGE;
                    if (type1 > type2) std::swap(type1, type2);
                    if(type1 == WALL_CANDIDATE && type2 == WALL_CANDIDATE){
                        type = WALL_WALL;
                    } else if(type1 == FLOOR_CANDIDATE && type2 == FLOOR_CANDIDATE){
                        type = FLOOR_FLOOR;
                    } else if(type1 == CEILING_CANDIDATE && type2 == CEILING_CANDIDATE){
                        type = CEILING_CEILING;
                    } else if(type1 == FLOOR_CANDIDATE && type2 == WALL_CANDIDATE){
                        type = FLOOR_WALL;
                    } else if(type1 == CEILING_CANDIDATE && type2 == WALL_CANDIDATE){
                        type = CEILING_WALL;
                    }
                    mpGraph[i*mNodenum + j] = type;
                    mpGraph[j*mNodenum + i] = type;
                    mEdgenum++;
                }
            }
        }


    }
    int mNodenum = 0;
    int mEdgenum = 0;
    int* mpGraph = nullptr;
    vector<Segment> mvSegments;
};

pcl::visualization::PCLVisualizer::Ptr VisualizeGraph (AdjacencyGraph& graph){
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    viewer->setBackgroundColor (0, 0, 0);
    std::vector<pcl::PointXYZ> centroids;
    
    for(auto& s: graph.mvSegments){
        if(s.mType == WALL_CANDIDATE){
            *cloud += *(s.mpCloud);
        }
    }

    for(int row = 0; row < graph.mNodenum-1; row++){
        for(int col = row+1; col < graph.mNodenum; col++){
            if(graph.mpGraph[row*graph.mNodenum + col]){
                viewer->addLine(graph.mvSegments[row].mCentroid, graph.mvSegments[col].mCentroid, "id_"+std::to_string(row*graph.mNodenum + col));
            }
        }
    }

    
    cout << "Number of Graph Nodes:" << graph.mNodenum << "\n";
    cout << "Number of Graph Edges:" << graph.mEdgenum << "\n";
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    
    return (viewer);
}


int main(int argc, char* argv[]){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if ( pcl::io::loadPCDFile <pcl::PointXYZ> (argv[1], *cloud) == -1) {
        std::cout << "Cloud reading failed." << std::endl;
        return (-1);
    }

    pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);    
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;

    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud);
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals);

    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 5.0);
    pass.filter (*indices);

    // TODO: Find parameters that make good clusters
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    std::vector <pcl::PointIndices> vClusterIndices;
    reg.setMinClusterSize (100);
    reg.setMaxClusterSize (1000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (300);
    reg.setInputCloud (cloud);
    //reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (1.0);
    reg.extract (vClusterIndices);  

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segment(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    
    pcl::ExtractIndices<pcl::PointXYZRGB> filter;
    pcl::PointXYZ centroid;
    

    

    
    std::vector<Segment> segments;
    AdjacencyGraph G;
    int cluster_id = 0;
    for (auto  c = vClusterIndices.begin(); c != vClusterIndices.end(); c++){
        // Generate Segment
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices(*c)); 
        filter.setInputCloud (colored_cloud);
        filter.setIndices(inliers);
        filter.filter(*segment);
        
        // Compute centroid of point cloud 
        pcl::computeCentroid(*segment, centroid);
        
        // Compute Normal Vector
        Eigen::Vector4f planeCoef;
        float curvature;
        pcl::computePointNormal (*segment, planeCoef, curvature);        
        double degree = pcl::getAngle3D(planeCoef.cwiseAbs(), Eigen::Vector4f(0, 1, 0, 0), true);  // Roughly computed angle

        Segment seg(*segment, planeCoef, curvature, centroid, 0);
        G.mvSegments.push_back(seg);
    }   

    G.MergeOverlappingSegments();
    G.ConstructGraph();
    pcl::visualization::PCLVisualizer::Ptr viewer(VisualizeGraph(G));
    
    while (!viewer->wasStopped ()) {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }


    return (0);
}
