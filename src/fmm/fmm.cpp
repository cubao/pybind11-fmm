#include "fmm.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace cubao {
namespace fmm {

// From topo_graph/fmm/fmm_algo.py:search_tr_cs_knn (line 527-556)
std::vector<std::vector<Candidate>> search_candidates(const Network& network, const RowVectors& trajectory,
                                                      const Config& config) {
    std::vector<std::vector<Candidate>> all_candidates;
    all_candidates.reserve(trajectory.rows());

    for (int i = 0; i < trajectory.rows(); ++i) {
        Eigen::Vector2d gps_pt = trajectory.row(i);

        // Query spatial index
        auto projected = network.query_radius(gps_pt, config.radius);

        // Sort by distance, keep top k
        std::sort(projected.begin(), projected.end(),
                  [](const auto& a, const auto& b) { return a.distance < b.distance; });

        if (projected.size() > static_cast<size_t>(config.k)) {
            projected.resize(config.k);
        }

        // Convert to Candidate format
        std::vector<Candidate> candidates;
        candidates.reserve(projected.size());
        for (const auto& pp : projected) {
            candidates.push_back({pp.edge_id, pp.offset, pp.distance, pp.point});
        }

        all_candidates.push_back(std::move(candidates));
    }

    return all_candidates;
}

// Helper: Emission probability (Gaussian)
// From topo_graph/fmm/transition_graph.py:calc_ep (line 172-178)
double calc_emission_prob(double dist, double gps_error) { return std::exp(-0.5 * std::pow(dist / gps_error, 2)); }

// Helper: Transition probability
// From topo_graph/fmm/transition_graph.py:calc_tp (line 164-170)
double calc_transition_prob(double eu_dist, double sp_dist) {
    if (sp_dist <= 0.0) {
        return 1.0;  // Same point or invalid
    }
    return (eu_dist >= sp_dist) ? 1.0 : (eu_dist / sp_dist);
}

// From topo_graph/fmm/fmm_algo.py:_update_tg (line 639-746)
// and topo_graph/fmm/transition_graph.py:backtrack (line 180-196)
MatchResult match_trajectory(const Network& network, const RowVectors& trajectory,
                             const std::vector<std::vector<Candidate>>& candidates, const Config& config) {
    const int N = trajectory.rows();
    if (N == 0 || candidates.empty()) {
        return {};
    }

    // Check if first layer has candidates
    if (candidates[0].empty()) {
        return {};
    }

    // Build transition graph (HMM trellis)
    // Each node represents a candidate at a specific GPS point
    struct TGNode {
        int layer;          // GPS point index
        int candidate_idx;  // Index in candidates[layer]
        double cum_prob;    // Cumulative log probability
        TGNode* prev;       // Backpointer for Viterbi (nullptr for first layer)
    };

    // Store nodes in a vector of vectors (one vector per layer)
    std::vector<std::vector<TGNode>> layers(N);

    // Initialize first layer (emission probabilities only)
    layers[0].reserve(candidates[0].size());
    for (size_t i = 0; i < candidates[0].size(); ++i) {
        double ep = calc_emission_prob(candidates[0][i].distance, config.gps_error);
        layers[0].push_back({0, static_cast<int>(i), std::log(ep), nullptr});
    }

    // Forward pass: compute transition probabilities
    for (int layer = 0; layer < N - 1; ++layer) {
        auto& curr_layer = layers[layer];
        auto& next_layer = layers[layer + 1];

        // Skip if current layer is empty
        if (curr_layer.empty() || candidates[layer + 1].empty()) {
            continue;
        }

        // Calculate euclidean distance between GPS points
        Eigen::Vector2d curr_gps = trajectory.row(layer);
        Eigen::Vector2d next_gps = trajectory.row(layer + 1);
        double eu_dist = (next_gps - curr_gps).norm();

        // Reserve space for next layer
        next_layer.reserve(candidates[layer + 1].size());

        // For each candidate in next layer
        for (size_t j = 0; j < candidates[layer + 1].size(); ++j) {
            const auto& next_cand = candidates[layer + 1][j];
            double ep = calc_emission_prob(next_cand.distance, config.gps_error);

            double best_prob = -std::numeric_limits<double>::infinity();
            TGNode* best_prev = nullptr;

            // For each candidate in current layer
            for (auto& curr_node : curr_layer) {
                const auto& curr_cand = candidates[layer][curr_node.candidate_idx];

                // Compute shortest path distance
                // TODO: Implement actual shortest path computation
                // For now, use euclidean distance as approximation
                double sp_dist = eu_dist;

                // Calculate transition probability
                double tp = calc_transition_prob(eu_dist, sp_dist);

                // Cumulative probability (in log space)
                double prob = curr_node.cum_prob + std::log(ep) + std::log(tp);

                if (prob > best_prob) {
                    best_prob = prob;
                    best_prev = &curr_node;
                }
            }

            next_layer.push_back({layer + 1, static_cast<int>(j), best_prob, best_prev});
        }
    }

    // Find best final node (Viterbi backtracking)
    TGNode* best_node = nullptr;
    double best_final_prob = -std::numeric_limits<double>::infinity();

    for (auto& node : layers[N - 1]) {
        if (node.cum_prob > best_final_prob) {
            best_final_prob = node.cum_prob;
            best_node = &node;
        }
    }

    if (!best_node) {
        return {};
    }

    // Backtrack to construct result
    MatchResult result;
    result.score = best_node->cum_prob;
    result.success = true;

    // Trace back through nodes
    TGNode* node = best_node;
    std::vector<MatchedCandidate> path;  // Reversed path

    while (node) {
        const auto& cand = candidates[node->layer][node->candidate_idx];
        path.push_back({
            cand.edge_id, cand.offset,
            std::exp(node->cum_prob)  // Convert log prob back to probability
        });
        node = node->prev;
    }

    // Reverse to get correct order
    result.matched_points.reserve(path.size());
    for (auto it = path.rbegin(); it != path.rend(); ++it) {
        result.matched_points.push_back(*it);
    }

    // Extract optimal path (unique edge IDs in sequence)
    for (const auto& mp : result.matched_points) {
        if (result.optimal_path.empty() || result.optimal_path.back() != mp.edge_id) {
            result.optimal_path.push_back(mp.edge_id);
        }
    }

    return result;
}

}  // namespace fmm
}  // namespace cubao
