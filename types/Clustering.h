#ifndef GIG_MODELISATION_GEOMETRIQUE_CLUSTERER_H
#define GIG_MODELISATION_GEOMETRIQUE_CLUSTERER_H
#include "VoxelGrid.h"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

class Clustering {
    std::vector<PointVectorPair> cloud;
    VoxelGrid voxel_grid;

public:
    explicit Clustering(
        const std::vector<PointVectorPair> &cloud,
        const VoxelGrid &voxel_grid
    ) : cloud(cloud), voxel_grid(voxel_grid) {
    }

    static Point find_centroid(
        const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > &point_cloud
    ) {
        pcl::PointXYZ temp{0.0f, 0.0f, 0.0f};
        const float num_points = static_cast<float>(point_cloud->size());

        for (const auto &point: *point_cloud) {
            temp.x += point.x;
            temp.y += point.y;
            temp.z += point.z;
        }

        return {
            temp.x / num_points,
            temp.y / num_points,
            temp.z / num_points
        };
    }

    static std::optional<PointVectorPair> process_point_cloud_once(
        pcl::SACSegmentation<pcl::PointXYZ> &segmentation,
        const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > &point_cloud,
        const int min_points_per_plane
    ) {
        if (point_cloud->size() < min_points_per_plane) {
            return std::nullopt;
        }

        segmentation.setInputCloud(point_cloud);

        const auto coefficients = std::make_shared<pcl::ModelCoefficients>();
        const auto inliers = std::make_shared<pcl::PointIndices>();
        segmentation.segment(*inliers, *coefficients);

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(point_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);

        const auto plane_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
        extract.filter(*plane_cloud);

        extract.setNegative(true);
        extract.filter(*point_cloud);

        const PointVectorPair point_vector{
            find_centroid(plane_cloud),
            Vector(
                coefficients->values[0],
                coefficients->values[1],
                coefficients->values[2]
            )
        };

        return point_vector;
    }

    std::map<VoxelKey, std::vector<PointVectorPair> > extract_planes_per_voxel(
        const float distance_threshold,
        const int max_iterations,
        const int min_points_per_plane
    ) {
        std::map<VoxelKey, std::vector<PointVectorPair> > elected_planes;

        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(distance_threshold);
        seg.setMaxIterations(max_iterations);

        for (const auto &[voxel_key, points]: voxel_grid.grid_points) {
            if (points.size() < min_points_per_plane) continue;

            std::vector<PointVectorPair> voxel_points_planes;
            const auto voxel_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();

            for (const auto &p: points | std::views::keys) {
                pcl::PointXYZ pcl_point;
                pcl_point.x = static_cast<float>(p.x());
                pcl_point.y = static_cast<float>(p.y());
                pcl_point.z = static_cast<float>(p.z());
                voxel_cloud->points.push_back(pcl_point);
            }

            while (voxel_cloud->size() >= min_points_per_plane) {
                auto maybe_elected_plane = process_point_cloud_once(seg, voxel_cloud, min_points_per_plane);
                if (!maybe_elected_plane.has_value()) break;
                const auto &elected_plane = maybe_elected_plane.value();

                voxel_points_planes.push_back(elected_plane);
                std::cout << "Found plane in voxel " << voxel_key.i << "," << voxel_key.j << "," << voxel_key.k
                        << " with centroid at (" << elected_plane.first.x() << ", "
                        << elected_plane.first.y() << ", " << elected_plane.first.z() << ")"
                        << " And coefficients: ["
                        << elected_plane.second.x() << ", "
                        << elected_plane.second.y() << ", "
                        << elected_plane.second.z() << "]"
                        << std::endl;
            }

            elected_planes[voxel_key] = voxel_points_planes;
        }

        return elected_planes;
    }
};

#endif //GIG_MODELISATION_GEOMETRIQUE_CLUSTERER_H
