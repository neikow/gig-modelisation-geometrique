#ifndef DATA_LOADER_H
#define DATA_LOADER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <filesystem>

#include <CGAL/IO/read_points.h>
#include "CICP_Types.h"
#include "VoxelGrid.h"

namespace fs = std::filesystem;

class DataLoader {
public:
    bool load(const std::string &filepath, std::vector<PointVectorPair> &cloud) {
        std::cout << "Loading file: " << filepath << "..." << std::endl;

        // Check if file exists
        if (!fs::exists(filepath)) {
            std::cerr << "Error: File does not exist: " << filepath << std::endl;
            return false;
        }

        // Get extension and convert to lowercase
        std::string ext = fs::path(filepath).extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

        cloud.clear();

        if (ext == ".csv") {
            return loadCSV(filepath, cloud);
        } else if (ext == ".ply" || ext == ".xyz") {
            return loadStandardCGAL(filepath, cloud);
        } else {
            std::cerr << "Error: Unsupported file extension '" << ext << "'" << std::endl;
            return false;
        }
    }


    void export_voxel_centers(const std::string &filepath, const VoxelGrid &grid) {
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

        for (const auto &[key, indices]: grid.grid_indices) {
            double cx = grid.min_x + (key.i + 0.5) * grid.voxel_size;
            double cy = grid.min_y + (key.j + 0.5) * grid.voxel_size;
            double cz = grid.min_z + (key.k + 0.5) * grid.voxel_size;

            out << cx << " " << cy << " " << cz << "\n";
        }
        std::cout << "Saved voxel debug cloud to " << filepath << std::endl;
    }

    void create_directories_for_file(const std::string &filepath) {
        const std::filesystem::path full_path(filepath);
        const std::filesystem::path parent_dir = full_path.parent_path();
        std::filesystem::create_directories(parent_dir);
    }

    void export_cloud_with_normals(const std::string &filepath, const std::vector<PointVectorPair> &cloud) {
        std::ofstream out(filepath);
        create_directories_for_file(filepath);

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

        for (const auto &pair: cloud) {
            const Point &p = pair.first;
            const Vector &n = pair.second;
            out << p.x() << " " << p.y() << " " << p.z() << " "
                    << n.x() << " " << n.y() << " " << n.z() << "\n";
        }
    }

private:
    bool loadCSV(const std::string &filepath, std::vector<PointVectorPair> &cloud) {
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

    bool loadStandardCGAL(const std::string &filepath, std::vector<PointVectorPair> &cloud) {
        bool success = CGAL::IO::read_points(filepath,
                                             std::back_inserter(cloud),
                                             CGAL::parameters::point_map(
                                                 CGAL::First_of_pair_property_map<PointVectorPair>()));

        if (!success) {
            std::cerr << "Error: CGAL failed to read file." << std::endl;
            return false;
        }

        std::cout << "Successfully loaded " << cloud.size() << " points (CGAL Standard)." << std::endl;
        return true;
    }
};

#endif
