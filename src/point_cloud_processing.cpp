#include "point_cloud_processing.h"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

PointCloudProcessing::PointCloudProcessing() 
    : tree_(new pcl::KdTreeFLANN<PointT>) {}

void PointCloudProcessing::loadPointCloud(const std::string &filename, PointCloudT::Ptr cloud) {
    if (pcl::io::loadPCDFile<PointT>(filename, *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s\n", filename.c_str());
    }
}

void PointCloudProcessing::savePointCloud(const std::string &filename, PointCloudT::Ptr cloud) {
    pcl::io::savePCDFileASCII(filename, *cloud);
}

void PointCloudProcessing::registerPointClouds(PointCloudT::Ptr source, PointCloudT::Ptr target, PointCloudT::Ptr output) {
    icp_.setInputSource(source);
    icp_.setInputTarget(target);
    icp_.align(*output);
}

void PointCloudProcessing::segmentPointCloud(PointCloudT::Ptr cloud, PointCloudT::Ptr segmented_cloud) {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg_.setInputCloud(cloud);
    seg_.setModelType(pcl::SACMODEL_PLANE);
    seg_.setMethodType(pcl::SAC_RANSAC);
    seg_.setDistanceThreshold(0.01);
    seg_.segment(*inliers, *coefficients);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*segmented_cloud);
}

void PointCloudProcessing::extractFeatures(PointCloudT::Ptr cloud, pcl::PointCloud<pcl::FPFHSignature33>::Ptr features) {
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    ne_.setInputCloud(cloud);
    ne_.setRadiusSearch(0.03);
    ne_.compute(*normals);

    fpfh_.setInputCloud(cloud);
    fpfh_.setInputNormals(normals);
    fpfh_.setRadiusSearch(0.05);
    fpfh_.compute(*features);
}

void PointCloudProcessing::classifyPointCloud(PointCloudT::Ptr cloud, pcl::PointCloud<pcl::FPFHSignature33>::Ptr features) {
    // Not implemented in this example
}

void PointCloudProcessing::detectObjects(PointCloudT::Ptr cloud, std::vector<PointCloudT::Ptr> &objects) {
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    euclidean_cluster_.setClusterTolerance(0.02);
    euclidean_cluster_.setMinClusterSize(100);
    euclidean_cluster_.setMaxClusterSize(25000);
    euclidean_cluster_.setSearchMethod(tree);
    euclidean_cluster_.setInputCloud(cloud);
    euclidean_cluster_.extract(cluster_indices);

    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        PointCloudT::Ptr cloud_cluster(new PointCloudT);
        for (const auto& idx : it->indices)
            cloud_cluster->points.push_back(cloud->points[idx]);
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        objects.push_back(cloud_cluster);
    }
}

void PointCloudProcessing::segmentRoad(PointCloudT::Ptr cloud, PointCloudT::Ptr road_cloud) {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg_.setInputCloud(cloud);
    seg_.setModelType(pcl::SACMODEL_PLANE);
    seg_.setMethodType(pcl::SAC_RANSAC);
    seg_.setDistanceThreshold(0.01);
    seg_.segment(*inliers, *coefficients);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*road_cloud);
}
