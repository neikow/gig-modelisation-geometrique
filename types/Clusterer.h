#ifndef CICP_CLUSTERER_H
#define CICP_CLUSTERER_H

#include <vector>
#include <map>
#include <cmath>
#include <limits>
#include <CGAL/squared_distance_3.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "Types.h"
#include "VoxelGrid.h"

namespace Clusterer {
    /**
     * Elect the representative point closest to the centroid of the given indices.
     * @param cloud The original point cloud
     * @param indices The indices of points in the cluster
     * @return The elected representative point
     */
    static pcl::PointXYZ elect_representative(
        const std::vector<PointVectorPair> &cloud,
        const std::vector<size_t> &indices
    ) {
        // Compute Centroid
        double sx = 0, sy = 0, sz = 0;
        for (const size_t idx: indices) {
            sx += cloud[idx].first.x();
            sy += cloud[idx].first.y();
            sz += cloud[idx].first.z();
        }
        const auto n = static_cast<double>(indices.size());
        const Point centroid(sx / n, sy / n, sz / n);

        // Find the closest point
        size_t best_idx = indices[0];
        double min_dist = std::numeric_limits<double>::max();

        for (const size_t idx: indices) {
            if (
                const double d = CGAL::squared_distance(centroid, cloud[idx].first);
                d < min_dist
            ) {
                min_dist = d;
                best_idx = idx;
            }
        }
        return {
            static_cast<float>(cloud[best_idx].first.x()),
            static_cast<float>(cloud[best_idx].first.y()),
            static_cast<float>(cloud[best_idx].first.z())
        };
    }

    /**
     * Determine the number of clusters (k) based on normal coherence.
     * @param normals The normals in the voxel
     * @return The number of clusters to use
     */
    static int determine_k(
        const std::vector<Vector> &normals
    ) {
        if (normals.size() < 5) return 1;
        Vector avg(0, 0, 0);
        for (const auto &n: normals) avg = avg + n;
        const double coherence = std::sqrt(avg.squared_length()) / normals.size();
        return (coherence < 0.9) ? 2 : 1;
    }

    /**
     * K-Means clustering on normals using cosine similarity.
     * @param indices The original indices of the points in the cloud
     * @param normals The normals corresponding to the indices
     * @param k The number of clusters
     * @param max_iterations Maximum iterations for convergence
     * @return A vector of clusters, each containing the original indices
     */
    [[nodiscard]] inline std::vector<std::vector<size_t> > run_k_means(
        const std::vector<size_t> &indices,
        const std::vector<Vector> &normals,
        const int k,
        const int max_iterations
    ) {
        if (k == 1) return {indices};
        std::vector centroids = {normals.front(), normals.back()};
        std::vector<std::vector<size_t> > clusters(k);

        for (int iter = 0; iter < max_iterations; ++iter) {
            std::vector<std::vector<size_t> > new_clusters(k);
            std::vector new_sums(k, Vector(0, 0, 0));
            std::vector counts(k, 0);

            for (size_t i = 0; i < normals.size(); ++i) {
                double best_sim = -2.0;
                int best_c = 0;
                for (int c = 0; c < k; ++c) {
                    if (
                        const double sim = normals[i] * centroids[c];
                        sim > best_sim
                    ) {
                        best_sim = sim;
                        best_c = c;
                    }
                }
                new_clusters[best_c].push_back(indices[i]);
                new_sums[best_c] = new_sums[best_c] + normals[i];
                counts[best_c]++;
            }

            bool changed = false;
            for (int c = 0; c < k; ++c) {
                if (counts[c] > 0) {
                    double len = std::sqrt(new_sums[c].squared_length());
                    Vector new_cent = new_sums[c] / len;
                    if (std::abs(new_cent.x() - centroids[c].x()) > 0.01) changed = true;
                    centroids[c] = new_cent;
                }
            }
            clusters = new_clusters;
            if (!changed) break;
        }
        return clusters;
    }

    /**
     * Process the point cloud within the voxel grid to elect representative points.
     * @param cloud The original point cloud with normals
     * @param voxel_grid The voxel grid partitioning
     * @param max_iterations Maximum iterations for K-Means
     * @return A point cloud of elected representative points
     */
    [[nodiscard]] inline pcl::PointCloud<pcl::PointXYZ>::Ptr process(
        const std::vector<PointVectorPair> &cloud,
        const VoxelGrid &voxel_grid,
        const int max_iterations
    ) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr reps(new pcl::PointCloud<pcl::PointXYZ>);
        reps->points.reserve(voxel_grid.grid_indices.size());

        for (const auto &indices: voxel_grid.grid_indices | std::views::values) {
            if (indices.empty()) continue;

            // 1. Gather Normals for this voxel
            std::vector<Vector> local_normals;
            local_normals.reserve(indices.size());
            for (const size_t idx: indices) local_normals.push_back(cloud[idx].second);

            // 2. Determine K
            const int k = determine_k(local_normals);

            // 3. Run K-Means
            auto clusters = run_k_means(indices, local_normals, k, max_iterations);

            // 4. Elect Representatives
            for (const auto &cluster_indices: clusters) {
                if (cluster_indices.empty()) continue;
                reps->points.push_back(elect_representative(cloud, cluster_indices));
            }
        }
        std::cout << "--- Processed Clusters ---" << std::endl;
        std::cout << "Reps Size: " << reps->size() << std::endl;
        return reps;
    }
};

#endif
