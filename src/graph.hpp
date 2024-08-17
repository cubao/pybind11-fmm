#pragma once

#include <any>

#include "types.hpp"

// https://github.com/cubao/networkx-graph/blob/b2ea16cd66dc77b4643a1a059c2496d44fef4532/src/main.cpp
namespace cubao {
struct DiGraph {};

struct Sinks {
    // you stop at (on) sinks, no passing through
    const DiGraph *graph{nullptr};
    unordered_set<int64_t> nodes;
};

using Binding = std::tuple<double, double, std::any>;  // start, end, data
struct Bindings {
    // you stop at first hit of bindings
    const DiGraph *graph{nullptr};
    unordered_map<int64_t, std::vector<Binding>> node2bindings;  // assume sorted
};

struct Sequences {
    const DiGraph *graph{nullptr};
    unordered_map<int64_t, std::vector<std::vector<int64_t>>> head2seqs;
    std::map<int, std::vector<std::vector<int64_t>>> search_in(const std::vector<int64_t> &nodes,
                                                               bool quick_return = true) const
};

struct Path {
    Path() = default;
    Path(const DiGraph *graph, double dist = 0.0, const std::vector<int64_t> &nodes = {},
         std::optional<double> start_offset = {}, std::optional<double> end_offset = {})
        : graph(graph), dist(dist), nodes(nodes), start_offset(start_offset), end_offset(end_offset) {}
    const DiGraph *graph{nullptr};
    double dist{0.0};
    std::vector<int64_t> nodes;
    std::optional<double> start_offset;
    std::optional<double> end_offset;
    std::optional<std::tuple<int64_t, Binding>> binding;

    void round(double scale) {
        dist = ROUND(dist, scale);
        if (start_offset) {
            start_offset = ROUND(*start_offset, scale);
        }
        if (end_offset) {
            end_offset = ROUND(*end_offset, scale);
        }
    }
};

}  // namespace cubao
