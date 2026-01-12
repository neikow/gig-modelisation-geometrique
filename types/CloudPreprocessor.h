#ifndef GIG_MODELISATION_GEOMETRIQUE_CLOUDPREPROCESSOR_H
#define GIG_MODELISATION_GEOMETRIQUE_CLOUDPREPROCESSOR_H
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
        std::vector<PointVectorPair> decimatedCloud = {};
        decimatedCloud.reserve(cloud.size() / factor);

        if (factor <= 1.0) return decimatedCloud;

        std::uniform_real_distribution<> dis(0.0, 1.0);
        for (const auto &pair: cloud) {
            if (dis(gen) < (1.0 / factor)) {
                decimatedCloud.push_back(pair);
            }
        }

        decimatedCloud.shrink_to_fit();

        return decimatedCloud;
    }



    /**
     * Add Gaussian noise to each point in the cloud.
     * @param cloud The cloud to add noise to
     * @param stddev The standard deviation of the Gaussian noise
     * @return A new point cloud with added noise
     */
     std::vector<PointVectorPair> add_noise(const std::vector<PointVectorPair> &cloud, const double stddev) {
        std::vector<PointVectorPair> noisyCloud = {};
        noisyCloud.reserve(cloud.size());

        std::normal_distribution<> dis(0.0, stddev);
        for (const auto &[point, vector]: cloud) {
            auto noisyPoint = Point(
                point.x() + dis(gen),
                point.y() + dis(gen),
                point.z() + dis(gen)
            );
            noisyCloud.push_back({noisyPoint, vector});
        }

        noisyCloud.shrink_to_fit();

        return noisyCloud;
    }

    /**
     * Rotate the point cloud by given angles around the X, Y, and Z axes.
     * @param cloud The cloud to rotate
     * @param angleX Rotation angle around the X axis (in radians)
     * @param angleY Rotation angle around the Y axis (in radians)
     * @param angleZ Rotation angle around the Z axis (in radians)
     * @return A new rotated point cloud
     */
     std::vector<PointVectorPair> rotate(
        const std::vector<PointVectorPair> &cloud,
        const double angleX,
        const double angleY,
        const double angleZ
    ) {
        std::vector<PointVectorPair> rotatedCloud = {};
        rotatedCloud.reserve(cloud.size());

        Eigen::Matrix3d rotX;
        rotX = Eigen::AngleAxisd(angleX, Eigen::Vector3d::UnitX());
        Eigen::Matrix3d rotY;
        rotY = Eigen::AngleAxisd(angleY, Eigen::Vector3d::UnitY());
        Eigen::Matrix3d rotZ;
        rotZ = Eigen::AngleAxisd(angleZ, Eigen::Vector3d::UnitZ());

        const Eigen::Matrix3d rotationMatrix = rotZ * rotY * rotX;

        for (const auto &[point, vector]: cloud) {
            Eigen::Vector3d p(point.x(), point.y(), point.z());
            Eigen::Vector3d rotatedP = rotationMatrix * p;

            auto rotatedPoint = Point(rotatedP.x(), rotatedP.y(), rotatedP.z());
            rotatedCloud.push_back({rotatedPoint, vector});
        }

        rotatedCloud.shrink_to_fit();

        return rotatedCloud;
    }

    /**
     * Translate the given cloud.
     * @param cloud The cloud to translate
     * @param transx The x for translation
     * @param transy The y for translation
     * @param transz The z for translation
     * @return A new point cloud translated
     */

     std::vector<PointVectorPair> translate(
        const std::vector<PointVectorPair> &cloud,
        const double transX,
        const double transY,
        const double transZ
    ) {
        std::vector<PointVectorPair> translatedCloud = {};
        translatedCloud.reserve(cloud.size());

        for (const auto &[point, vector]: cloud) {
            auto translatedPoint = Point(
                point.x() + transX,
                point.y() + transY,
                point.z() + transZ
            );
            translatedCloud.push_back({translatedPoint, vector});
        }

        translatedCloud.shrink_to_fit();

        return translatedCloud;
    }
};

#endif //GIG_MODELISATION_GEOMETRIQUE_CLOUDPREPROCESSOR_H
