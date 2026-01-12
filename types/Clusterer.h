#ifndef CLUSTERER_H
#define CLUSTERER_H

#include <vector>
#include <map>
#include <cmath>
#include <limits>
#include <CGAL/centroid.h>
#include <CGAL/squared_distance_3.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "CICP_Types.h"
#include "VoxelGrid.h" 

class Clusterer {
private:
    // Clustering Parameters could go here (e.g., max_kmeans_iter)
    int max_iterations = 5;

public:
    explicit Clusterer() {}

    // The main function takes data as input, not constructor
    pcl::PointCloud<pcl::PointXYZ>::Ptr process(
        const std::vector<PointVectorPair>& cloud,
        const VoxelGrid& voxel_grid
    ) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr reps(new pcl::PointCloud<pcl::PointXYZ>);
        reps->points.reserve(voxel_grid.grid_indices.size());

        for(const auto& [key, indices] : voxel_grid.grid_indices) {
            if(indices.empty()) continue;

            // 1. Gather Normals for this voxel
            std::vector<Vector> local_normals;
            local_normals.reserve(indices.size());
            for(size_t idx : indices) local_normals.push_back(cloud[idx].second);

            // 2. Determine K
            int k = determineK(local_normals);

            // 3. Run K-Means
            auto clusters = runKMeans(indices, local_normals, k);

            // 4. Elect Representatives
            for(const auto& cluster_indices : clusters) {
                if(cluster_indices.empty()) continue;
                reps->points.push_back(electRepresentative(cloud, cluster_indices));
            }
        }
        std::cout << "--- Processed Clusters ---" << std::endl;
        std::cout << "Reps Size: " << reps->size() << std::endl;
        return reps;
    }

private:
    pcl::PointXYZ electRepresentative(const std::vector<PointVectorPair>& cloud
        , const std::vector<size_t>& indices) {
        // Compute Centroid
        double sx = 0, sy = 0, sz = 0;
        for(size_t idx : indices) {
            sx += cloud[idx].first.x();
            sy += cloud[idx].first.y();
            sz += cloud[idx].first.z();
        }
        double n = static_cast<double>(indices.size());
        Point centroid(sx/n, sy/n, sz/n);

        // Find closest point
        size_t best_idx = indices[0];
        double min_dist = std::numeric_limits<double>::max();

        for(size_t idx : indices) {
            double d = CGAL::squared_distance(centroid, cloud[idx].first);
            if(d < min_dist) {
                min_dist = d;
                best_idx = idx;
            }
        }
        return pcl::PointXYZ(cloud[best_idx].first.x(), cloud[best_idx].first.y(), cloud[best_idx].first.z());
    }

    int determineK(const std::vector<Vector>& normals) {
        if(normals.size() < 5) return 1; 
        Vector avg(0,0,0);
        for(const auto& n : normals) avg = avg + n;
        double coherence = std::sqrt(avg.squared_length()) / normals.size();
        return (coherence < 0.9) ? 2 : 1;
    }

    std::vector<std::vector<size_t>> runKMeans(const std::vector<size_t>& indices, const std::vector<Vector>& normals, int k) {
        if(k==1) return {indices};
        std::vector<Vector> centroids = {normals.front(), normals.back()};
        std::vector<std::vector<size_t>> clusters(k);

        for(int iter=0; iter < max_iterations; ++iter) {
            std::vector<std::vector<size_t>> new_clusters(k);
            std::vector<Vector> new_sums(k, Vector(0,0,0));
            std::vector<int> counts(k, 0);

            for(size_t i=0; i<normals.size(); ++i) {
                double best_sim = -2.0; int best_c = 0;
                for(int c=0; c<k; ++c) {
                    double sim = normals[i] * centroids[c];
                    if(sim > best_sim) { best_sim = sim; best_c = c; }
                }
                new_clusters[best_c].push_back(indices[i]);
                new_sums[best_c] = new_sums[best_c] + normals[i];
                counts[best_c]++;
            }

            bool changed = false;
            for(int c=0; c<k; ++c) {
                if(counts[c] > 0) {
                    double len = std::sqrt(new_sums[c].squared_length());
                    Vector new_cent = new_sums[c] / len;
                    if(std::abs(new_cent.x() - centroids[c].x()) > 0.01) changed = true;
                    centroids[c] = new_cent;
                }
            }
            clusters = new_clusters;
            if(!changed) break;
        }
        return clusters;
    }
};

#endif