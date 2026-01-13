#ifndef CICP_CLOUD_PREPROCESSOR_H
#define CICP_CLOUD_PREPROCESSOR_H
#include <random>
#include <eigen3/Eigen/Dense>

class CloudPreprocessor {
    int seed = 0;
    std::mt19937 gen;

public:
    CloudPreprocessor() : gen(seed) {
    }

    /**
     * Decimate the point cloud by randomly removing points based on the given factor.
     * @param cloud The cloud to decimate
     * @param factor The decimation factor (e.g., 2 means keep around half the points)
     * @return A new decimated point cloud
     */
    std::vector<PointVectorPair> decimate(const std::vector<PointVectorPair> &cloud, const double factor) {
        std::vector<PointVectorPair> decimated_cloud = {};
        decimated_cloud.reserve(cloud.size() / factor);

        if (factor <= 1.0) return decimated_cloud;

        std::uniform_real_distribution<> dis(0.0, 1.0);
        for (const auto &pair: cloud) {
            if (dis(gen) < (1.0 / factor)) {
                decimated_cloud.push_back(pair);
            }
        }

        decimated_cloud.shrink_to_fit();

        return decimated_cloud;
    }

    /**
     * Add Gaussian noise to each point in the cloud.
     * @param cloud The cloud to add noise to
     * @param stddev The standard deviation of the Gaussian noise
     * @return A new point cloud with added noise
     */
    std::vector<PointVectorPair> add_noise(const std::vector<PointVectorPair> &cloud, const double stddev) {
        std::vector<PointVectorPair> noisy_cloud = {};
        noisy_cloud.reserve(cloud.size());

        std::normal_distribution<> dis(0.0, stddev);
        for (const auto &[point, vector]: cloud) {
            auto noisy_point = Point(
                point.x() + dis(gen),
                point.y() + dis(gen),
                point.z() + dis(gen)
            );
            noisy_cloud.push_back({noisy_point, vector});
        }

        noisy_cloud.shrink_to_fit();

        return noisy_cloud;
    }

    /**
     * Rotate the point cloud by given angles around the X, Y, and Z axes.
     * @param cloud The cloud to rotate
     * @param angle_x Rotation angle around the X axis (in radians)
     * @param angle_y Rotation angle around the Y axis (in radians)
     * @param angle_z Rotation angle around the Z axis (in radians)
     * @return A new rotated point cloud
     */
    static std::vector<PointVectorPair> rotate(
        const std::vector<PointVectorPair> &cloud,
        const double angle_x,
        const double angle_y,
        const double angle_z
    ) {
        std::vector<PointVectorPair> rotated_cloud = {};
        rotated_cloud.reserve(cloud.size());

        Eigen::Matrix3d rot_x;
        rot_x = Eigen::AngleAxisd(angle_x, Eigen::Vector3d::UnitX());
        Eigen::Matrix3d rot_y;
        rot_y = Eigen::AngleAxisd(angle_y, Eigen::Vector3d::UnitY());
        Eigen::Matrix3d rot_z;
        rot_z = Eigen::AngleAxisd(angle_z, Eigen::Vector3d::UnitZ());

        const Eigen::Matrix3d rotation_matrix = rot_z * rot_y * rot_x;

        for (const auto &[point, vector]: cloud) {
            Eigen::Vector3d p(point.x(), point.y(), point.z());
            Eigen::Vector3d rotated_p = rotation_matrix * p;

            auto rotatedPoint = Point(rotated_p.x(), rotated_p.y(), rotated_p.z());
            rotated_cloud.push_back({rotatedPoint, vector});
        }

        rotated_cloud.shrink_to_fit();

        return rotated_cloud;
    }

    /**
     * Translate the given cloud.
     * @param cloud The cloud to translate
     * @param trans_x The x for translation
     * @param trans_y The y for translation
     * @param trans_z The z for translation
     * @return A new point cloud translated
     */
    static std::vector<PointVectorPair> translate(
        const std::vector<PointVectorPair> &cloud,
        const double trans_x,
        const double trans_y,
        const double trans_z
    ) {
        std::vector<PointVectorPair> translated_cloud = {};
        translated_cloud.reserve(cloud.size());

        for (const auto &[point, vector]: cloud) {
            auto translated_point = Point(
                point.x() + trans_x,
                point.y() + trans_y,
                point.z() + trans_z
            );
            translated_cloud.emplace_back(translated_point, vector);
        }

        translated_cloud.shrink_to_fit();

        return translated_cloud;
    }
};

#endif