#ifndef CICP_REGISTRATION_HELPERS_H
#define CICP_REGISTRATION_HELPERS_H

#include <vector>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/transformation_estimation_svd.h>

namespace RegistrationHelpers {
    /**
     * Perform one alignment step between source and target point clouds.
     * @param source The source point cloud
     * @param target The target point cloud
     * @param rejection_dist_sq The squared distance threshold for rejecting correspondences
     * @return The estimated transformation matrix (4x4)
     */
    inline Eigen::Matrix4f align_step(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &target,
        const double rejection_dist_sq
    ) {
        if (source->empty() || target->empty()) return Eigen::Matrix4f::Identity();

        // 1. Build KD-Tree for Target
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(target);

        // 2. Find Correspondences
        pcl::Correspondences correspondences;
        std::vector<int> idx(1);
        std::vector<float> sq_dist(1);

        for (size_t i = 0; i < source->size(); ++i) {
            if (kdtree.nearestKSearch(source->at(i), 1, idx, sq_dist) > 0) {
                if (sq_dist[0] < rejection_dist_sq) {
                    pcl::Correspondence corr;
                    corr.index_query = static_cast<int>(i);
                    corr.index_match = idx[0];
                    correspondences.push_back(corr);
                }
            }
        }

        std::cout << "  [Align] Found " << correspondences.size() << " valid correspondences." << std::endl;

        if (correspondences.size() < 3) {
            std::cout << "  [Align] WARNING: Too few correspondences (<3). Returning Identity." << std::endl;
            return Eigen::Matrix4f::Identity();
        }

        // 3. SVD
        pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> estimator;
        Eigen::Matrix4f transformation;

        estimator.estimateRigidTransformation(
            *source,
            *target,
            correspondences,
            transformation
        );

        return transformation;
    }
};

#endif
