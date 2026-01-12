#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <vector>
#include <string>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "CICP_Types.h"

class Visualizer {
public:
    static void visualize(
        const std::vector<PointVectorPair>& dense,
        const std::vector<PointVectorPair>& sparse_start,
        const std::vector<PointVectorPair>& sparse_end
    ) {
        // Convert to PCL Format
        auto pcl_dense = toPCL(dense);
        auto pcl_start = toPCL(sparse_start);
        auto pcl_end   = toPCL(sparse_end);

        // 2. Setup Viewer
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("CICP Result Viewer"));
        viewer->setBackgroundColor(0.05, 0.05, 0.05); // Dark Grey Background
        viewer->initCameraParameters();

        // Add Dense Cloud (Target) -> GRAY
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> gray(pcl_dense, 180, 180, 180);
        viewer->addPointCloud<pcl::PointXYZ>(pcl_dense, gray, "dense_cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "dense_cloud");

        // Add Sparse Cloud before registration  -> RED
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(pcl_start, 255, 0, 0);
        viewer->addPointCloud<pcl::PointXYZ>(pcl_start, red, "start_cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "start_cloud");

        // Add Sparse Cloud after registration  -> BLUE
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue(pcl_end, 0, 100, 255);
        viewer->addPointCloud<pcl::PointXYZ>(pcl_end, blue, "end_cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "end_cloud");

        std::cout << "\n[Visualizer] Window Open." << std::endl;
        std::cout << "  - Gray: Dense Target" << std::endl;
        std::cout << "  - Red:  Sparse Before Alignment" << std::endl;
        std::cout << "  - Blue: Sparse After Alignment" << std::endl;
        std::cout << "Press 'q' in the window to close it." << std::endl;

        // Loop
        while (!viewer->wasStopped()) {
            viewer->spinOnce(100);
        }
    }

private:
    static pcl::PointCloud<pcl::PointXYZ>::Ptr toPCL(const std::vector<PointVectorPair>& cloud) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl_cloud->points.reserve(cloud.size());
        
        for(const auto& pair : cloud) {
            pcl_cloud->points.push_back(pcl::PointXYZ(
                (float)pair.first.x(), 
                (float)pair.first.y(), 
                (float)pair.first.z()
            ));
        }
        return pcl_cloud;
    }
};

#endif