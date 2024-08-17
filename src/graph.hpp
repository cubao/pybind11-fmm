#pragma once

#include <any>

#include "types.hpp"

// https://github.com/cubao/networkx-graph/blob/b2ea16cd66dc77b4643a1a059c2496d44fef4532/src/main.cpp
namespace cubao {
struct DiGraph;

struct Sinks {
    // you stop at (on) sinks, no passing through
    const DiGraph *graph{nullptr};
    unordered_set<int64_t> nodes;
    unordered_map<int64_t, std::vector<int64_t>> links;
};

using Binding = std::tuple<double, double, std::any>;  // start, end, data
struct Bindings {
    // you stop at first hit of bindings
    const DiGraph *graph{nullptr};
    unordered_map<int64_t, std::vector<Binding>> node2bindings;  // assume sorted
    void sort();
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
};

struct ZigzagPath : Path {
    // two-way routing on a DiGraph
    ZigzagPath() = default;
    ZigzagPath(const DiGraph *graph, double dist, const std::vector<int64_t> &nodes, const std::vector<int> &directions)
        : Path(graph, dist, nodes), directions(directions) {}
    std::vector<int> directions;
};

struct ShortestPathGenerator {
    const DiGraph *graph{nullptr};
    unordered_map<int64_t, int64_t> prevs;
    unordered_map<int64_t, double> dists;

    using Click = std::tuple<int64_t, std::optional<double>>;
    double cutoff{0.0};
    std::optional<Click> source;
    std::optional<Click> target;
    bool ready() const { return graph && cutoff > 0 && ((bool)source ^ (bool)target); }
};

struct ZigzagPathGenerator {
    using State = std::tuple<int64_t, int>;
    ZigzagPathGenerator() = default;
    ZigzagPathGenerator(const DiGraph *graph, double cutoff) : graph(graph), cutoff(cutoff) {}

    const DiGraph *graph{nullptr};
    double cutoff{0.0};
    std::optional<int64_t> source;
    unordered_map<State, State> prevs;
    unordered_map<State, double> dists;

    bool ready() const { return graph && cutoff > 0 && source; }

    static std::optional<ZigzagPath> Path(const State &state, const int64_t source,  //
                                          const DiGraph *self,                       //
                                          const unordered_map<State, State> &pmap,
                                          const unordered_map<State, double> &dmap);
};

// https://github.com/cubao/nano-fmm/blob/master/src/nano_fmm/network/ubodt.hpp
struct UbodtRecord {
    UbodtRecord() {}
    UbodtRecord(int64_t source_road, int64_t target_road,  //
                int64_t source_next, int64_t target_prev,  //
                double cost)
        : source_road(source_road),
          target_road(target_road),  //
          source_next(source_next),
          target_prev(target_prev),  //
          cost(cost) {}

    bool operator<(const UbodtRecord &rhs) const {
        if (source_road != rhs.source_road) {
            return source_road < rhs.source_road;
        }
        if (cost != rhs.cost) {
            return cost < rhs.cost;
        }
        return std::make_tuple(source_next, target_prev, target_road) <
               std::make_tuple(rhs.source_next, rhs.target_prev, rhs.target_road);
    }
    bool operator==(const UbodtRecord &rhs) const {
        return source_road == rhs.source_road && target_road == rhs.target_road && source_next == rhs.source_next &&
               target_prev == rhs.target_prev && cost == rhs.cost;
    }

    int64_t source_road{0};
    int64_t target_road{0};
    int64_t source_next{0};
    int64_t target_prev{0};
    double cost{0.0};
};

struct DiGraph {};

}  // namespace cubao
