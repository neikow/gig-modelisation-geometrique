#ifndef CICP_PIPELINE_H
#define CICP_PIPELINE_H

#include <iostream>
#include <vector>
#include <string>

#include "Types.h"
#include "DataLoader.h"
#include "CloudPreprocessor.h"
#include "NormalEstimator.h"
#include "VoxelGrid.h"
#include "Clusterer.h"
#include "RegistrationHelpers.h"

/** CICP Pipeline Class */
class CICP {
public:
    VoxelGrid voxel_grid;

private:
    std::vector<PointVectorPair> dense_cloud;
    std::vector<PointVectorPair> sparse_cloud;
    std::vector<PointVectorPair> sparse_cloud_initial;

    int registration_max_iterations;
    double voxel_size;
    int normal_estimation_k_neighbors; 
    int max_clusterer_iterations = 10;
    double initial_rejection_sq = 25;
    double min_rejection_sq = 0.01;
    double dynamic_threshold_decay = 0.96;

public:
    /** Constructor for CICP Pipeline
     * @param v_size Voxel size for voxel grid
     * @param r_max_iterations Maximum iterations for registration
     * @param normal_estimation_k_neighbors Number of neighbors for normal estimation
     */
    explicit CICP(
        const double v_size = 2.0,
        const int r_max_iterations = 50,
        const int normal_estimation_k_neighbors = 10,
        const int cluster_max_iter = 10
    )
        : voxel_grid(v_size),
          registration_max_iterations(r_max_iterations),
          voxel_size(v_size),
          max_clusterer_iterations(cluster_max_iter),
          normal_estimation_k_neighbors(normal_estimation_k_neighbors) {
    }

    /** Load Dense Cloud (Target) */
    bool load_data(const std::string &filepath) {
        std::cout << "[Loader] Loading Dense Cloud..." << std::endl;
        return DataLoader::load(filepath, dense_cloud);
    }

    /** Load Sparse Cloud (Source) */
    bool load_sparse_data(const std::string &filepath) {
        std::cout << "[Loader] Loading Sparse Cloud..." << std::endl;
        return DataLoader::load(filepath, sparse_cloud);
    }

    /** Preprocess the dense cloud to create a sparse cloud by decimation and perturbation.
     * @param decimation_factor The factor by which to decimate the dense cloud
     * @param translation_x Translation in x direction
     * @param translation_y Translation in y direction
     * @param translation_z Translation in z direction
     * @param rotation_x_rad Rotation around x axis in radians
     * @param rotation_y_rad Rotation around y axis in radians
     * @param rotation_z_rad Rotation around z axis in radians
     */
    void generate_synthetic_sparse_cloud(
        const double decimation_factor = 20.0,
        const double translation_x = 1.0,
        const double translation_y = -2.0,
        const double translation_z = 0.5,
        const double rotation_x_rad = 5.0 * M_PI / 180.0,
        const double rotation_y_rad = -3.0 * M_PI / 180.0,
        const double rotation_z_rad = 10.0 * M_PI / 180.0
    ) {
        CloudPreprocessor preprocessor;
        std::cout << "[Preprocess] Decimating and Perturbing..." << std::endl;
        sparse_cloud = preprocessor.decimate(
            dense_cloud,
            decimation_factor
        );

        sparse_cloud = preprocessor.translate(
            sparse_cloud,
            translation_x,
            translation_y,
            translation_z
        );
        sparse_cloud = preprocessor.rotate(
            sparse_cloud,
            rotation_x_rad,
            rotation_y_rad,
            rotation_z_rad
        );
    }

    /** Estimate normals for both dense and sparse clouds. */
    void estimate_normals() {
        if (dense_cloud.empty()) {
            std::cerr << "Error: Dense cloud empty during normal estimation!" << std::endl;
            return;
        }
        NormalEstimator::compute(dense_cloud, normal_estimation_k_neighbors);

        if (sparse_cloud.empty()) {
            std::cerr << "Error: Sparse cloud empty. Did you forget to load it or run preprocess?" << std::endl;
            return;
        }
        NormalEstimator::compute(sparse_cloud, normal_estimation_k_neighbors);
    }



   /** Run the CICP registration process.
     * @param output_path The path to save the registered sparse cloud.
     */
    void run_registration(const std::string &output_path) {
        if (dense_cloud.empty() || sparse_cloud.empty()) return;
        sparse_cloud_initial = sparse_cloud;
        
        std::cout << "--- Preparing Target (Dense) ---" << std::endl;
        voxel_grid.voxel_size = voxel_size;
        voxel_grid.create(dense_cloud);
        const auto target_reps = Clusterer::process(dense_cloud, voxel_grid, max_clusterer_iterations);

        std::cout << "--- Starting Loop ---" << std::endl;
        Eigen::Matrix4f global_transform = Eigen::Matrix4f::Identity();

        // Use Dynamic Threshold
        double current_rejection = initial_rejection_sq; 

        for (int i = 0; i < registration_max_iterations; ++i) {
            voxel_grid.create(sparse_cloud);
            auto source_reps = Clusterer::process(sparse_cloud, voxel_grid, max_clusterer_iterations);

            // Check Convergence
            Eigen::Matrix4f delta = RegistrationHelpers::align_step(source_reps, target_reps, current_rejection);
            const double diff = (delta - Eigen::Matrix4f::Identity()).norm();
            if(i % 10 == 0 || i == registration_max_iterations-1)
                std::cout << "Iter " << i << " | Delta: " << diff << " | Rej: " << current_rejection << std::endl;

            if (current_rejection <= min_rejection_sq + 1e-9 && diff < 1e-4) {
                std::cout << "--> Converged." << std::endl;
                break;
            }

            global_transform = delta * global_transform;
            apply_transform(sparse_cloud, delta);

            // UPDATE DYNAMIC THRESHOLD
            current_rejection = std::max(current_rejection * dynamic_threshold_decay, min_rejection_sq);
        }

        compute_final_metrics();
        DataLoader::export_cloud_with_normals(output_path, sparse_cloud);
    }

    [[nodiscard]] const std::vector<PointVectorPair> &get_dense() const { return dense_cloud; }
    [[nodiscard]] const std::vector<PointVectorPair> &get_sparse_initial() const { return sparse_cloud_initial; }
    [[nodiscard]] const std::vector<PointVectorPair> &get_sparse_final() const { return sparse_cloud; }

private:

  void compute_final_metrics() {
        if (dense_cloud.empty() || sparse_cloud.empty()) return;

        std::cout << "\n--- Computing Final Registration Metrics ---" << std::endl;

        // 1. Convert Dense Cloud to PCL for KD-Tree
        pcl::PointCloud<pcl::PointXYZ>::Ptr dense_pcl(new pcl::PointCloud<pcl::PointXYZ>);
        dense_pcl->points.reserve(dense_cloud.size());
        for (const auto &pair : dense_cloud) {
            dense_pcl->points.emplace_back(
                static_cast<float>(pair.first.x()),
                static_cast<float>(pair.first.y()),
                static_cast<float>(pair.first.z())
            );
        }

        // 2. Build Tree
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(dense_pcl);

        // 3. Query Every Sparse Point
        double total_sq_error = 0.0;
        double total_abs_error = 0.0;
        std::vector<int> indices(1);
        std::vector<float> sq_dists(1);

        for (const auto &pair : sparse_cloud) {
            pcl::PointXYZ query(
                static_cast<float>(pair.first.x()),
                static_cast<float>(pair.first.y()),
                static_cast<float>(pair.first.z())
            );

            if (kdtree.nearestKSearch(query, 1, indices, sq_dists) > 0) {
                total_sq_error += sq_dists[0];
                total_abs_error += std::sqrt(sq_dists[0]);
            }
        }

        // 4. Calculate Stats
        double n = static_cast<double>(sparse_cloud.size());
        double rmse = std::sqrt(total_sq_error / n);
        double mean_error = total_abs_error / n;

        std::cout << "  > Final RMSE:          " << rmse << " units" << std::endl;
        std::cout << "  > Mean Distance Error: " << mean_error << " units" << std::endl;
        std::cout << "------------------------------------------\n" << std::endl;
    }

    /** Apply the given transformation to the point cloud.
     * @param cloud The cloud to transform
     * @param trans The transformation matrix (4x4)
     */
    static void apply_transform(std::vector<PointVectorPair> &cloud, const Eigen::Matrix4f &trans) {
        Eigen::Matrix4d T = trans.cast<double>();
        const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
        const Eigen::Vector3d t = T.block<3, 1>(0, 3);

        for (auto &[pt, vec]: cloud) {
            Eigen::Vector3d p(pt.x(), pt.y(), pt.z());
            Eigen::Vector3d n(vec.x(), vec.y(), vec.z());

            p = R * p + t;
            n = R * n;

            pt = Point(p.x(), p.y(), p.z());
            vec = Vector(n.x(), n.y(), n.z());
        }
    }
};

#endif

