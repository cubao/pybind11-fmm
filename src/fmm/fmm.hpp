#pragma once

#include "../network.hpp"
#include "types.hpp"

namespace cubao {
namespace fmm {

// Candidate search (from topo_graph/fmm/fmm_algo.py:search_tr_cs_knn)
std::vector<std::vector<Candidate>> search_candidates(const Network& network,
                                                      const RowVectors& trajectory,  // Nx2 GPS points
                                                      const Config& config);

// HMM matching (from topo_graph/fmm/fmm_algo.py:_update_tg and transition_graph.py:backtrack)
MatchResult match_trajectory(const Network& network, const RowVectors& trajectory,
                             const std::vector<std::vector<Candidate>>& candidates, const Config& config);

// Helper functions
double calc_emission_prob(double dist, double gps_error);
double calc_transition_prob(double eu_dist, double sp_dist);

}  // namespace fmm
}  // namespace cubao
