#ifndef CICP_NORMAL_ESTIMATOR_H
#define CICP_NORMAL_ESTIMATOR_H

#include <vector>
#include <iostream>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/property_map.h>
#include "Types.h"

/** Normal Estimator Namespace */
namespace NormalEstimator {
    /** Compute normals for the given point cloud using PCA.
     * @param cloud The point cloud to compute normals for
     * @param k_neighbors The number of neighbors to use for normal estimation
     */
    inline void compute(std::vector<PointVectorPair> &cloud, const int k_neighbors) {
        if (cloud.empty()) return;

        std::cout << "[NormalEstimator] Computing with k=" << k_neighbors << "..." << std::endl;

        CGAL::pca_estimate_normals<CGAL::Parallel_if_available_tag>(
            cloud,
            k_neighbors,
            CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>())
            .normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>())
        );

        // Post-process: Normalize to ensure unit vectors
        for (auto &normal: cloud | std::views::values) {
            double norm = std::sqrt(normal.squared_length());
            if (norm > 1e-15) normal = normal / norm;
            else normal = Vector(0, 0, 1);
        }
    }
};

#endif
