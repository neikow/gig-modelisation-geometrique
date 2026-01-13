#ifndef CICP_VOXEL_GRID_H
#define CICP_VOXEL_GRID_H

#include <vector>
#include <map>
#include <iostream>
#include "Types.h"

struct VoxelKey {
    long i, j, k;

    bool operator<(const VoxelKey &other) const {
        if (i != other.i) return i < other.i;
        if (j != other.j) return j < other.j;
        return k < other.k;
    }
};

/** Voxel Grid Class */
class VoxelGrid {
public:
    std::map<VoxelKey, std::vector<size_t> > grid_indices;
    std::map<VoxelKey, std::vector<PointVectorPair> > grid_points;

    double voxel_size;

    double min_x = std::numeric_limits<double>::max(), max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max(), max_y = std::numeric_limits<double>::lowest();
    double min_z = std::numeric_limits<double>::max(), max_z = std::numeric_limits<double>::lowest();

    explicit VoxelGrid(const double size) : voxel_size(size) {
    }

    /** Create the voxel grid from the given point cloud.
     * @param cloud The point cloud to voxelize
     */
    void create(const std::vector<PointVectorPair> &cloud) {
        if (cloud.empty()) return;

        const Point &p0 = cloud[0].first;
        min_x = max_x = p0.x();
        min_y = max_y = p0.y();
        min_z = max_z = p0.z();

        for (const auto &key: cloud | std::views::keys) {
            const Point &p = key;
            if (p.x() < min_x) min_x = p.x();
            if (p.x() > max_x) max_x = p.x();
            if (p.y() < min_y) min_y = p.y();
            if (p.y() > max_y) max_y = p.y();
            if (p.z() < min_z) min_z = p.z();
            if (p.z() > max_z) max_z = p.z();
        }

        grid_indices.clear();
        grid_points.clear();

        for (size_t idx = 0; idx < cloud.size(); ++idx) {
            const auto &p = cloud[idx].first;
            const auto i = static_cast<long>((p.x() - min_x) / voxel_size);
            const auto j = static_cast<long>((p.y() - min_y) / voxel_size);
            const auto k = static_cast<long>((p.z() - min_z) / voxel_size);

            VoxelKey key = {i, j, k};
            grid_indices[key].push_back(idx);
            grid_points[key].push_back(cloud[idx]);
        }

        std::cout << "Voxelization complete. Created " << grid_indices.size() << " voxels." << std::endl;
    }

    /** Get the number of voxels in the grid.
     * @return The number of voxels
     */
    [[nodiscard]] size_t size() const {
        return grid_indices.size();
    }
};

#endif
