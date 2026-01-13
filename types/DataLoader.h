#ifndef CICP_DATA_LOADER_H
#define CICP_DATA_LOADER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <filesystem>

#include <CGAL/IO/read_points.h>
#include "Types.h"
#include "VoxelGrid.h"

namespace fs = std::filesystem;

namespace DataLoader {
    /** Create directories for the given file path if they do not exist.
     * @param filepath The file path for which to create directories
     */
    inline void create_directories_for_file(const std::string &filepath) {
        const std::filesystem::path full_path(filepath);
        const std::filesystem::path parent_dir = full_path.parent_path();
        std::filesystem::create_directories(parent_dir);
    }

    /** Load point cloud from a CSV file.
     * Expects each line to contain three floating-point numbers (x, y, z).
     * @param filepath The path to the CSV file
     * @param cloud The output point cloud
     * @return True if loading was successful, false otherwise
     */
    inline bool load_csv(
        const std::string &filepath,
        std::vector<PointVectorPair> &cloud
    ) {
        std::ifstream stream(filepath);
        if (!stream) {
            std::cerr << "Error: Could not open CSV file: " << filepath << std::endl;
            return false;
        }

        std::string line;
        while (std::getline(stream, line)) {
            if (line.empty()) continue;

            std::stringstream ss(line);
            double x, y, z;

            if (ss >> x >> y >> z) {
                cloud.push_back({Point(x, y, z), Vector(0, 0, 0)});
            }
        }

        if (cloud.empty()) {
            std::cerr << "Error: No points found in CSV." << std::endl;
            return false;
        }

        std::cout << "Successfully loaded " << cloud.size() << " points (CSV)." << std::endl;
        return true;
    }

    /** Load point cloud using CGAL's standard reader for supported formats (PLY, XYZ).
     * @param filepath The path to the point cloud file
     * @param cloud The output point cloud
     * @return True if loading was successful, false otherwise
     */
    inline bool load_standard_cgal(const std::string &filepath, std::vector<PointVectorPair> &cloud) {
        const bool success = CGAL::IO::read_points(
            filepath,
            std::back_inserter(cloud),
            CGAL::parameters::point_map(
                CGAL::First_of_pair_property_map<PointVectorPair>()
            )
        );

        if (!success) {
            std::cerr << "Error: CGAL failed to read file." << std::endl;
            return false;
        }

        std::cout << "Successfully loaded " << cloud.size() << " points (CGAL Standard)." << std::endl;
        return true;
    }

    /** Load point cloud from file, dispatching based on file extension.
     * Supports CSV, PLY, and XYZ formats.
     * @param filepath The path to the point cloud file
     * @param cloud The output point cloud
     * @return True if loading was successful, false otherwise
     */
    inline bool load(const std::string &filepath, std::vector<PointVectorPair> &cloud) {
        std::cout << "Loading file: " << filepath << "..." << std::endl;

        // Check if file exists
        if (!fs::exists(filepath)) {
            std::cerr << "Error: File does not exist: " << filepath << std::endl;
            return false;
        }

        // Get extension and convert to lowercase
        std::string ext = fs::path(filepath).extension().string();
        std::ranges::transform(ext, ext.begin(), ::tolower);

        cloud.clear();

        if (ext == ".csv") {
            return load_csv(filepath, cloud);
        }

        if (ext == ".ply" || ext == ".xyz") {
            return load_standard_cgal(filepath, cloud);
        }

        std::cerr << "Error: Unsupported file extension '" << ext << "'" << std::endl;
        return false;
    }

    /** Export voxel centers to a PLY file for debugging.
     * @param filepath The path to the output PLY file
     * @param grid The voxel grid containing voxel information
     */
    inline void export_voxel_centers(const std::string &filepath, const VoxelGrid &grid) {
        create_directories_for_file(filepath);
        std::ofstream out(filepath);
        if (!out) return;

        // PLY Header
        out << "ply\n";
        out << "format ascii 1.0\n";
        out << "element vertex " << grid.grid_indices.size() << "\n";
        out << "property float x\n";
        out << "property float y\n";
        out << "property float z\n";
        out << "end_header\n";

        for (const auto &[i, j, k]: grid.grid_indices | std::views::keys) {
            const double cx = grid.min_x + (i + 0.5) * grid.voxel_size;
            const double cy = grid.min_y + (j + 0.5) * grid.voxel_size;
            const double cz = grid.min_z + (k + 0.5) * grid.voxel_size;

            out << cx << " " << cy << " " << cz << "\n";
        }
        std::cout << "Saved voxel debug cloud to " << filepath << std::endl;
    }

    /** Export point cloud with normals to a PLY file.
     * @param filepath The path to the output PLY file
     * @param cloud The point cloud with normals to export
     */
    inline void export_cloud_with_normals(const std::string &filepath, const std::vector<PointVectorPair> &cloud) {
        create_directories_for_file(filepath);
        std::ofstream out(filepath);

        if (!out) {
            std::cerr << "Error: Could not open file: " << filepath << std::endl;
            return;
        };

        out << "ply\n";
        out << "format ascii 1.0\n";
        out << "element vertex " << cloud.size() << "\n";
        out << "property float x\n";
        out << "property float y\n";
        out << "property float z\n";
        out << "property float nx\n";
        out << "property float ny\n";
        out << "property float nz\n";
        out << "end_header\n";

        for (const auto &[p, n]: cloud) {
            out << p.x() << " " << p.y() << " " << p.z() << " "
                    << n.x() << " " << n.y() << " " << n.z() << "\n";
        }
    }
};

#endif
