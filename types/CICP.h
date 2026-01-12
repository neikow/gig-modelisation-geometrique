#ifndef CICP_PIPELINE_H
#define CICP_PIPELINE_H

#include <iostream>
#include <vector>
#include <string>

// Include component headers
#include "CICP_Types.h"
#include "DataLoader.h"
#include "CloudPreprocessor.h"
#include "NormalEstimator.h"
#include "VoxelGrid.h"
#include "Clusterer.h"
#include "RegistrationHelpers.h"

class CICP {
public:
    DataLoader dataLoader;
    CloudPreprocessor preprocessor;
    NormalEstimator normalEstimator;
    VoxelGrid voxelGrid;
    Clusterer clusterer;

private:
    std::vector<PointVectorPair> denseCloud;
    std::vector<PointVectorPair> sparseCloud;
    std::vector<PointVectorPair> sparseCloudInitial;
    
    int registration_max_iterations;
    double voxel_size;
    double rejection_sq = 25.0;

public:
    CICP(double vSize = 2.0, int r_max_iterations = 50, int k_neighbors = 10) 
        : voxelGrid(vSize), normalEstimator(k_neighbors), voxel_size(vSize), 
          registration_max_iterations(r_max_iterations) {}

    // Load Dense Cloud (Target)
    bool loadData(const std::string& filepath) {
        std::cout << "[Loader] Loading Dense Cloud..." << std::endl;
        return dataLoader.load(filepath, denseCloud);
    }

    // NEW: Load Sparse Cloud (Source) directly from file
    bool loadSparseData(const std::string& filepath) {
        std::cout << "[Loader] Loading Sparse Cloud..." << std::endl;
        return dataLoader.load(filepath, sparseCloud);
    }

    // Generates sparse cloud from dense (if no file provided)
    void preprocess() {
        std::cout << "[Preprocess] Decimating and Perturbing..." << std::endl;
        sparseCloud = preprocessor.decimate(denseCloud, 20.0);
        
        sparseCloud = preprocessor.translate(sparseCloud, 1.0, -2.0, 0.5);
        double d2r = M_PI / 180.0;
        sparseCloud = preprocessor.rotate(sparseCloud, 5.0*d2r, -3.0*d2r, 10.0*d2r);
    }

    void estimateNormals() {
        if (denseCloud.empty()) {
            std::cerr << "Error: Dense cloud empty during normal estimation!" << std::endl;
            return;
        }
        normalEstimator.compute(denseCloud);
        
        if (sparseCloud.empty()) {
            std::cerr << "Error: Sparse cloud empty. Did you forget to load it or run preprocess?" << std::endl;
            return;
        }
        normalEstimator.compute(sparseCloud);
    }

    void runRegistration(const std::string& output_path) {
        if(denseCloud.empty() || sparseCloud.empty()) return;
        sparseCloudInitial = sparseCloud;
        std::cout << "--- Preparing Target (Dense) ---" << std::endl;
        voxelGrid.voxel_size = voxel_size; 
        voxelGrid.create(denseCloud);
        auto targetReps = clusterer.process(denseCloud, voxelGrid);

        std::cout << "--- Starting Loop ---" << std::endl;
        Eigen::Matrix4f global_transform = Eigen::Matrix4f::Identity();

        for(int i=0; i<registration_max_iterations; ++i) {
            voxelGrid.create(sparseCloud); 
            auto sourceReps = clusterer.process(sparseCloud, voxelGrid);

            Eigen::Matrix4f delta = RegistrationHelpers::alignStep(sourceReps, targetReps, rejection_sq);
            double diff = (delta - Eigen::Matrix4f::Identity()).norm();
            
            std::cout << "Iter " << i << " | Delta: " << diff << std::endl;
            if(diff < 1e-4) break;

            global_transform = delta * global_transform;
            applyTransform(sparseCloud, delta);
        }
        dataLoader.export_cloud_with_normals(output_path, sparseCloud);
    }

    const std::vector<PointVectorPair>& getDense() const { return denseCloud; }
    const std::vector<PointVectorPair>& getSparseInitial() const { return sparseCloudInitial; }
    const std::vector<PointVectorPair>& getSparseFinal() const { return sparseCloud; }

private:
    void applyTransform(std::vector<PointVectorPair>& cloud, const Eigen::Matrix4f& trans) {
        Eigen::Matrix4d T = trans.cast<double>();
        Eigen::Matrix3d R = T.block<3,3>(0,0);
        Eigen::Vector3d t = T.block<3,1>(0,3);

        for(auto& pair : cloud) {
            Eigen::Vector3d p(pair.first.x(), pair.first.y(), pair.first.z());
            Eigen::Vector3d n(pair.second.x(), pair.second.y(), pair.second.z());
            
            p = R * p + t;
            n = R * n;

            pair.first = Point(p.x(), p.y(), p.z());
            pair.second = Vector(n.x(), n.y(), n.z());
        }
    }
};

#endif