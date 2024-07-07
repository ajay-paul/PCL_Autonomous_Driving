#include "point_cloud_processing.h"
#include <iostream>
#include <vector>

int main() {
    PointCloudProcessing::PointCloudT::Ptr source_cloud(new PointCloudProcessing::PointCloudT);
    PointCloudProcessing::PointCloudT::Ptr target_cloud(new PointCloudProcessing::PointCloudT);
    PointCloudProcessing::PointCloudT::Ptr registered_cloud(new PointCloudProcessing::PointCloudT);
    PointCloudProcessing::PointCloudT::Ptr segmented_cloud(new PointCloudProcessing::PointCloudT);
    PointCloudProcessing::PointCloudT::Ptr road_cloud(new PointCloudProcessing::PointCloudT);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features(new pcl::PointCloud<pcl::FPFHSignature33>);
    std::vector<PointCloudProcessing::PointCloudT::Ptr> detected_objects;

    PointCloudProcessing pcp;

    // Load point clouds
    pcp.loadPointCloud("data/office1.pcd", source_cloud);
    pcp.loadPointCloud("data/office2.pcd", target_cloud);

    // Point Cloud Registration
    pcp.registerPointClouds(source_cloud, target_cloud, registered_cloud);
    pcp.savePointCloud("data/registered.pcd", registered_cloud);

    // Road Segmentation
    pcp.segmentRoad(registered_cloud, road_cloud);
    pcp.savePointCloud("data/road.pcd", road_cloud);

    // Object Detection
    pcp.detectObjects(registered_cloud, detected_objects);
    int object_id = 0;
    for (auto &object : detected_objects) {
        std::string filename = "data/object_" + std::to_string(object_id++) + ".pcd";
        pcp.savePointCloud(filename, object);
    }

    // Feature Extraction
    #pragma omp parallel sections
    {
        #pragma omp section
        {
            pcp.extractFeatures(registered_cloud, source_features);
        }
        #pragma omp section
        {
            pcp.extractFeatures(target_cloud, target_features);
        }
    }

    // Classification (Note: Placeholder function)
    // pcp.classifyPointCloud(registered_cloud, source_features);
    // pcp.classifyPointCloud(target_cloud, target_features);

    return 0;
}
