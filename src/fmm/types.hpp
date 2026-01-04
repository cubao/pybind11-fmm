#pragma once

#include <vector>

#include "../types.hpp"

namespace cubao {
namespace fmm {

// Configuration (from topo_graph/fmm/fmm_algo.py:FastMapMatchConfig)
struct Config {
    int k = 50;                      // Max candidates per point
    double radius = 160.0;           // Search radius (meters)
    double gps_error = 40.0;         // GPS std deviation (meters)
    double reverse_tolerance = 0.0;  // Reverse path tolerance
};

// Candidate point (from topo_graph/fmm/transition_graph.py:Candidate)
struct Candidate {
    int64_t edge_id;
    double offset;          // Distance along edge
    double distance;        // Perpendicular distance to GPS point
    Eigen::Vector2d point;  // Projected point coordinates
};

// Matched result for single GPS point
struct MatchedCandidate {
    int64_t edge_id;
    double offset;
    double probability;
};

// Final matching result (from topo_graph/fmm/mm_type.py:MatchResult)
struct MatchResult {
    std::vector<MatchedCandidate> matched_points;  // One per GPS point
    std::vector<int64_t> optimal_path;             // Edge IDs of matched path
    double score;                                  // Log probability
    bool success;
};

}  // namespace fmm
}  // namespace cubao
