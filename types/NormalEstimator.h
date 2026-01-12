#ifndef NORMAL_ESTIMATOR_H
#define NORMAL_ESTIMATOR_H

#include <vector>
#include <iostream>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/property_map.h>
#include "CICP_Types.h"

class NormalEstimator {
private:
    int k_neighbors;

public:
    explicit NormalEstimator(int k = 10) : k_neighbors(k) {}

    void setK(int k) { k_neighbors = k; }

    void compute(std::vector<PointVectorPair>& cloud) {
        if (cloud.empty()) return;

        std::cout << "[NormalEstimator] Computing with k=" << k_neighbors << "..." << std::endl;

        CGAL::pca_estimate_normals<CGAL::Parallel_if_available_tag>(
            cloud,
            k_neighbors,
            CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>())
                             .normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>())
        );

        // Post-process: Normalize to ensure unit vectors
        for(auto& pair : cloud) {
            double len = std::sqrt(pair.second.squared_length());
            if(len > 1e-15) pair.second = pair.second / len;
            else pair.second = Vector(0,0,1);
        }
    }
};

#endif