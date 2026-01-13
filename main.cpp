#include <iostream>
#include <string>
#include "types/CICP.h"
#include "types/Visualizer.h"

// Constants
double VOXEL_SIZE = 0.08;
int REGISTRATION_MAX_ITERATIONS = 100;
int K_NEIGHBORS = 10;

int main(int argc, char **argv) {
    // Basic Argument Check
    if (argc < 3) {
        std::cerr << "Usage Error." << std::endl;
        std::cerr << "Usage: ./main input_dense.ply [input_sparse.ply] output_dir" << std::endl;
        std::cerr <<
                "       If input_sparse.ply is omitted, synthetic sparse cloud will be generated from the input cloud."
                << std::endl;
        return 1;
    }

    // Variables for Paths
    std::string input_dense_path;
    std::string input_sparse_path;
    std::string output_dir;
    bool is_sparse_provided = false;

    if (argc == 4) {
        // Expecting: [dense] [sparse] [output]
        input_dense_path = argv[1];
        input_sparse_path = argv[2];
        output_dir = argv[3];
        is_sparse_provided = true;
    } else {
        // Expecting: [dense] [output]
        input_dense_path = argv[1];
        output_dir = argv[2];
    }

    // Normalize output directory
    if (output_dir.back() != '/' && output_dir.back() != '\\') output_dir += "/";

    std::cout << "=== CICP Configuration ===" << std::endl;
    std::cout << "Mode: " << (is_sparse_provided ? "Two Files (External Sparse)" : "One File (Synthetic Sparse)") <<
            std::endl;
    std::cout << "Dense Input: " << input_dense_path << std::endl;
    if (is_sparse_provided) std::cout << "Sparse Input: " << input_sparse_path << std::endl;
    std::cout << "Output Dir:  " << output_dir << std::endl;

    // Instantiate Pipeline
    CICP cicp_pipeline(VOXEL_SIZE, REGISTRATION_MAX_ITERATIONS, K_NEIGHBORS);

    // Load Data
    // Always load dense
    if (!cicp_pipeline.load_data(input_dense_path)) return -1;

    if (is_sparse_provided) {
        // Load the provided sparse cloud
        if (!cicp_pipeline.load_sparse_data(input_sparse_path)) return -1;
        // NOTE: We do NOT run preprocess() here because we assume the 
        // loaded files are the ones we want to register directly.
    } else {
        // No sparse file provided -> Generate it from dense
        cicp_pipeline.generate_synthetic_sparse_cloud();
    }

    // Run Pipeline
    cicp_pipeline.estimate_normals();

    cicp_pipeline.run_registration(output_dir + "final_aligned.ply");

    std::cout << "CICP Pipeline Complete." << std::endl;

    Visualizer::visualize(
        cicp_pipeline.get_dense(),
        cicp_pipeline.get_sparse_initial(),
        cicp_pipeline.get_sparse_final()
    );

    return 0;
}
