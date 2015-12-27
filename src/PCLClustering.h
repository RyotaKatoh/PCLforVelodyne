//
//  PCLClustering.hpp
//  pcapVisualizer_example
//
//  Created by Ryota Katoh on 12/21/15.
//
//

#ifndef PCLClustering_hpp
#define PCLClustering_hpp

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

class PCLCluster{
public:
    typedef pcl::PointCloud<pcl::PointXYZI> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;
    typedef typename Cloud::Ptr CloudPtr;
    
    
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    pcl::PointIndices::Ptr inliers;
    pcl::ModelCoefficients::Ptr coefficients;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane;
    
    PCLCluster(): inliers(new pcl::PointIndices), coefficients(new pcl::ModelCoefficients), cloud_plane(new Cloud){
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(0.02); // maybe this is very important.
    };
    
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr createPCLCluster(CloudConstPtr &cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr createPCLCluster(CloudPtr &cloud);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr segment(CloudPtr &cloud, float threshold);

    
};

#endif /* PCLClustering_hpp */
