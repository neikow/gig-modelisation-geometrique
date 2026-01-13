#ifndef CICP_VISUALIZER_H
#define CICP_VISUALIZER_H

#include <vector>
#include <string>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "Types.h"

namespace Visualizer {
    /** Convert a point cloud from CGAL format to PCL format.
     * @param cloud The input point cloud in CGAL format
     * @return The converted point cloud in PCL format
     */
    static pcl::PointCloud<pcl::PointXYZ>::Ptr to_pcl(const std::vector<PointVectorPair> &cloud) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl_cloud->points.reserve(cloud.size());

        for (const auto &point: cloud | std::views::keys) {
            pcl_cloud->points.emplace_back(
                static_cast<float>(point.x()),
                static_cast<float>(point.y()),
                static_cast<float>(point.z())
            );
        }
        return pcl_cloud;
    }

    /** Add a point cloud to the viewer with specified color and point size.
     * @param viewer The PCL visualizer
     * @param cloud The point cloud to add
     * @param id The identifier for the cloud in the viewer
     * @param r Red color component (0-255)
     * @param g Green color component (0-255)
     * @param b Blue color component (0-255)
     * @param point_size The size of the points
     */
    static void add_point_cloud(
        const pcl::visualization::PCLVisualizer::Ptr &viewer,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        const std::string &id,
        const int r,
        const int g,
        const int b,
        const int point_size
    ) {
        const pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cloud, r, g, b);
        viewer->addPointCloud<pcl::PointXYZ>(cloud, color_handler, id);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, id);
    }

    /** Visualize the dense and sparse point clouds.
     * @param dense The dense target cloud
     * @param sparse_start The sparse cloud before registration
     * @param sparse_end The sparse cloud after registration
     */
    static void visualize(
        const std::vector<PointVectorPair> &dense,
        const std::vector<PointVectorPair> &sparse_start,
        const std::vector<PointVectorPair> &sparse_end
    ) {
        // Convert to PCL Format
        const auto pcl_dense = to_pcl(dense);
        const auto pcl_start = to_pcl(sparse_start);
        const auto pcl_end = to_pcl(sparse_end);

        // 2. Setup Viewer
        const pcl::visualization::PCLVisualizer::Ptr
                viewer(new pcl::visualization::PCLVisualizer(
                        "CICP Result Viewer")
                );

        viewer->setBackgroundColor(0.05, 0.05, 0.05); // Dark Grey Background
        viewer->initCameraParameters();

        // Add Dense Cloud (Target) -> GRAY
        add_point_cloud(viewer, pcl_dense, "dense_cloud", 180, 180, 180, 1);

        // Add Sparse Cloud before registration  -> RED
        add_point_cloud(viewer, pcl_start, "start_cloud", 255, 0, 0, 3);

        // Add Sparse Cloud after registration  -> BLUE
        add_point_cloud(viewer, pcl_end, "end_cloud", 0, 0, 255, 3);

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
};

#endif
