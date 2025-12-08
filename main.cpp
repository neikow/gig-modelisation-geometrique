#include <iostream>
#include <fstream>
#include <CGAL/IO/read_points.h>
#include <CGAL/pca_estimate_normals.h>
// created types
#include "types/CICP_Types.h"
#include "types/DataLoader.h"
#include "types/VoxelGrid.h"
#include "types/CloudPreprocessor.h"

int main(int argc, char **argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << "main file_path output_dir" << std::endl;
        return -1;
    }

    const std::string filename = argv[1];
    std::string output_dir = argv[2];

    std::cout << "Input file: " << filename << std::endl;
    std::cout << "Output directory: " << output_dir << std::endl;

    if (!output_dir.ends_with("/")) {
        output_dir += "/";
    }

    /// Load Data
    std::vector<PointVectorPair> sparseCloud;
    std::vector<PointVectorPair> denseCloud;

    DataLoader loader;

    if (!loader.load(filename, denseCloud)) {
        return -1;
    }

    /// Copie et prétraitement
    CloudPreprocessor preprocessor;
    // Décimation
    constexpr double decimationFactor = 20; // à ajuster
    sparseCloud = preprocessor.decimate(denseCloud, decimationFactor);
    std::cout << "Decimated cloud to " << sparseCloud.size() << " points." << std::endl;

    constexpr double noiseStdDev = 0.5; // à ajuster
    sparseCloud = preprocessor.add_noise(sparseCloud, noiseStdDev);
    std::cout << "Added Gaussian noise : `sd = " << noiseStdDev << "`." << std::endl;

    constexpr double dX = 1.0;
    constexpr double dY = -2.0;
    constexpr double dZ = 0.5;
    sparseCloud = preprocessor.translate(sparseCloud, dX, dY, dZ);
    std::cout << "Translated cloud by (" << dX << ", " << dY << ", " << dZ << ")." << std::endl;

    constexpr double angleX = 5.0 / 180 * boost::math::constants::pi<double>();
    constexpr double angleY = -3.0 / 180 * boost::math::constants::pi<double>();
    constexpr double angleZ = 10.0 / 180 * boost::math::constants::pi<double>();

    sparseCloud = preprocessor.rotate(sparseCloud, angleX, angleY, angleZ);
    std::cout << "Rotated cloud by (" << angleX << ", " << angleY << ", " << angleZ << ") degrees." << std::endl;

    /// Estimation de normal (� modifier pour ajouter elbow method)

    const int kDense = 5;
    const int kSparse = 3;

    std::cout << "Computing normals using PCA..." << std::endl;

    CGAL::pca_estimate_normals<CGAL::Parallel_if_available_tag>(
        denseCloud,
        kDense,
        CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>())
        .normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>())
    );

    CGAL::pca_estimate_normals<CGAL::Parallel_if_available_tag>(
        sparseCloud,
        kSparse,
        CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>())
        .normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>())
    );

    std::cout << "Computed " << denseCloud.size() << " normals." << std::endl;
    std::cout << "Computed " << sparseCloud.size() << " normals." << std::endl;

    // Voxelization
    double voxelSize = 0.5; // � ajouter une m�thode de calcul de taille de voxel
    std::cout << "Voxelizing with size " << voxelSize << "..." << std::endl;

    VoxelGrid vGridDense(voxelSize);
    VoxelGrid vGridSparse(voxelSize);
    vGridDense.create(denseCloud);
    vGridSparse.create(sparseCloud);

    // Debug
    /*if (!vGrid.grid.empty()) {
        long firstVoxelId = vGrid.grid.begin()->first;
        size_t count = vGrid.grid.begin()->second.size();
        std::cout << "Voxel ID " << firstVoxelId << " contains " << count << " points." << std::endl;
    }*/

    loader.export_cloud_with_normals(output_dir + "debug_cloud_dense.ply", denseCloud);
    loader.export_cloud_with_normals(output_dir + "debug_cloud_sparse.ply", sparseCloud);
    loader.export_voxel_centers(output_dir + "debug_voxels_dense.ply", vGridDense);
    loader.export_voxel_centers(output_dir + "debug_voxels_sparse.ply", vGridSparse);

    return 0;
}
