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
    if (argc < 4) {
        std::cerr << "Usage Error." << std::endl;
        std::cerr << "Case 1 (Two files): ./main true input_dense.ply input_sparse.ply output_dir" << std::endl;
        std::cerr << "Case 2 (One file):  ./main false input_dense.ply output_dir" << std::endl;
        return -1;
    }

    // Parse Flag (First Argument)
    std::string flag_str = argv[1];
    bool is_sparse_provided = (flag_str == "true" || flag_str == "1" || flag_str == "yes");

    // Variables for Paths
    std::string input_dense_path;
    std::string input_sparse_path;
    std::string output_dir;

    // Parse Remaining Arguments based on Flag
    if (is_sparse_provided) {
        // Expecting: [dense] [sparse] [output]
        if (argc < 5) {
            std::cerr << "Error: Flag is true, but sparse file argument is missing." << std::endl;
            return -1;
        }
        input_dense_path = argv[2];
        input_sparse_path = argv[3];
        output_dir = argv[4];
    } else {
        // Expecting: [dense] [output]
        input_dense_path = argv[2];
        output_dir = argv[3];
    }

    // Normalize output directory
    if (output_dir.back() != '/' && output_dir.back() != '\\') output_dir += "/";

    std::cout << "=== CICP Configuration ===" << std::endl;
    std::cout << "Mode: " << (is_sparse_provided ? "Two Files (External Sparse)" : "One File (Synthetic Sparse)") << std::endl;
    std::cout << "Dense Input: " << input_dense_path << std::endl;
    if(is_sparse_provided) std::cout << "Sparse Input: " << input_sparse_path << std::endl;
    std::cout << "Output Dir:  " << output_dir << std::endl;

    // Instantiate Pipeline
    CICP cicpPipeline(VOXEL_SIZE, REGISTRATION_MAX_ITERATIONS, K_NEIGHBORS);

    // Load Data
    // Always load dense
    if (!cicpPipeline.loadData(input_dense_path)) return -1;

    if (is_sparse_provided) {
        // Load the provided sparse cloud
        if (!cicpPipeline.loadSparseData(input_sparse_path)) return -1;
        // NOTE: We do NOT run preprocess() here because we assume the 
        // loaded files are the ones we want to register directly.
    } else {
        // No sparse file provided -> Generate it from dense
        cicpPipeline.preprocess(); 
    }

    // Run Pipeline
    cicpPipeline.estimateNormals();
    
    cicpPipeline.runRegistration(output_dir + "final_aligned.ply");

    std::cout << "CICP Pipeline Complete." << std::endl;

    Visualizer::visualize(
        cicpPipeline.getDense(),
        cicpPipeline.getSparseInitial(),
        cicpPipeline.getSparseFinal()
    );

    return 0;
}