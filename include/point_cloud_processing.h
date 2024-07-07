#ifndef POINT_CLOUD_PROCESSING_H
#define POINT_CLOUD_PROCESSING_H

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/transforms.h>
#include <omp.h>

class PointCloudProcessing {
public:
    using PointT = pcl::PointXYZ;
    using PointCloudT = pcl::PointCloud<PointT>;

    PointCloudProcessing();

    void loadPointCloud(const std::string &filename, PointCloudT::Ptr cloud);
    void savePointCloud(const std::string &filename, PointCloudT::Ptr cloud);

    void registerPointClouds(PointCloudT::Ptr source, PointCloudT::Ptr target, PointCloudT::Ptr output);
    void segmentPointCloud(PointCloudT::Ptr cloud, PointCloudT::Ptr segmented_cloud);
    void extractFeatures(PointCloudT::Ptr cloud, pcl::PointCloud<pcl::FPFHSignature33>::Ptr features);
    void classifyPointCloud(PointCloudT::Ptr cloud, pcl::PointCloud<pcl::FPFHSignature33>::Ptr features);
    void detectObjects(PointCloudT::Ptr cloud, std::vector<PointCloudT::Ptr> &objects);
    void segmentRoad(PointCloudT::Ptr cloud, PointCloudT::Ptr road_cloud);

private:
    pcl::IterativeClosestPoint<PointT, PointT> icp_;
    pcl::SACSegmentation<PointT> seg_;
    pcl::NormalEstimation<PointT, pcl::Normal> ne_;
    pcl::FPFHEstimation<PointT, pcl::Normal, pcl::FPFHSignature33> fpfh_;
    pcl::KdTreeFLANN<PointT>::Ptr tree_;
    pcl::EuclideanClusterExtraction<PointT> euclidean_cluster_;
};

#endif // POINT_CLOUD_PROCESSING_H
