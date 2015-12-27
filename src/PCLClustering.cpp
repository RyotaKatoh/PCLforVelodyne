//
//  PCLClustering.cpp
//  pcapVisualizer_example
//
//  Created by Ryota Katoh on 12/21/15.
//
//

#include "PCLClustering.h"

using namespace std;


pcl::PointCloud<pcl::PointXYZI>::Ptr PCLCluster::segment(CloudPtr &cloud, float threshold){

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_segmented(new pcl::PointCloud<pcl::PointXYZI>);
    *cloud_segmented = *cloud;
    
    int nr_points = (int)cloud->points.size();
    
    // cloud_plane and cloud_f are used for extract.
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZI>);
    
    while(cloud_segmented->points.size() > threshold * nr_points){
        seg.setInputCloud(cloud_segmented);
        seg.segment(*inliers, *coefficients);
        
        if(inliers->indices.size() == 0){
            cout<<"Could not estimate a planar model for the given dataset."<<endl;
            break;
        }
        
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(cloud_segmented);
        extract.setIndices(inliers);
        
        // extract positive cloud
        extract.setNegative(false);
        extract.filter(*cloud_plane);
        
        // extract negative cloud
        extract.setNegative(true);
        extract.filter(*cloud_f);
        
        *cloud_segmented = *cloud_f;
        
    }
    
    return cloud_segmented;
    
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCLCluster::createPCLCluster(CloudPtr &cloud){
    
    
    // cloud_filtered is filterd PC of input.
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    *cloud_filtered = *cloud;
    
    // if you needed, segment point cloud
   // cloud_filtered = segment(cloud, 0.4);

    cout<<"cloud_filtered(after): "<<cloud_filtered->points.size()<<endl;
    
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud_filtered);
    
    vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(0.3);
    //ec.setClusterTolerance(0.02);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);
    
    // color point cloud by cluster
    //pcl::PointCloud<pcl::PointXYZRGBA>
    cout<<"====cluster_indices======: "<<cluster_indices.size()<<endl;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    vector<uint32_t> colors(cluster_indices.size());
    
    for(size_t i=0;i<colors.size();i++){
    
        uint8_t r, g, b;
        switch (i%3) {
            case 0:
                r = 127 / (i/3 + 1) + 127;
                g = 255 - 127 / (i/3+1);
                b = 12;
                break;
                
            case 1:
                r = 12;
                g = 127 / (i/3 + 1) + 127;
                b = 255 - 127 * (i/3+1);
                break;
                
            case 2:
                r = 255 - 127 * (i/3+1);
                g = 12;
                b = 127 / (i/3+1) + 127;
                break;
        }
        
        
        uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
        colors[i] = rgb;
    }
    
    for(int i=0;i<cloud_filtered->points.size(); ++i) {
        pcl::PointXYZRGB color_point;
        color_point.x = cloud_filtered->points[i].x;
        color_point.y = cloud_filtered->points[i].y;
        color_point.z = cloud_filtered->points[i].z;
        color_point_cloud->points.push_back(color_point);
    }
    
    for(int i=0;i<colors.size();++i) {
        for(int j=0;j<cluster_indices[i].indices.size();++j){
            int idx = cluster_indices[i].indices[j];
            color_point_cloud->points[idx].rgb = *reinterpret_cast<float *>(&colors[i]);
        }
    }
    
    return color_point_cloud;
    

}

