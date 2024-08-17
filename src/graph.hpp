#pragma once

#include <any>

#include "types.hpp"

template <class T>
inline void hash_combine(std::size_t &seed, const T &value) {
    seed ^= std::hash<T>()(value) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

namespace std {
template <typename... T>
struct hash<tuple<T...>> {
    size_t operator()(const tuple<T...> &t) const {
        size_t seed = 0;
        hash_tuple(seed, t);
        return seed;
    }

  private:
    template <std::size_t I = 0, typename... Ts>
    inline typename enable_if<I == sizeof...(Ts), void>::type hash_tuple(size_t &seed, const tuple<Ts...> &t) const {}

    template <std::size_t I = 0, typename... Ts>
        inline typename enable_if < I<sizeof...(Ts), void>::type hash_tuple(size_t &seed, const tuple<Ts...> &t) const {
        hash_combine(seed, get<I>(t));
        hash_tuple<I + 1, Ts...>(seed, t);
    }
};
}  // namespace std

// https://github.com/cubao/networkx-graph/blob/b2ea16cd66dc77b4643a1a059c2496d44fef4532/src/main.cpp
namespace cubao {
struct DiGraph;

// routing on graph, you sink/stop at (on) sinks, no passing through
struct Sinks {
    const DiGraph *graph{nullptr};
    unordered_set<int64_t> nodes;
    unordered_map<int64_t, std::vector<int64_t>> links;
};

// routing on graph, you may stop (finish) on binding
using Binding = std::tuple<double, double, std::any>;  // start, end, data
struct Bindings {
    // you stop at first hit of bindings
    const DiGraph *graph{nullptr};
    unordered_map<int64_t, std::vector<Binding>> node2bindings;  // assume sorted
    void sort();
};

// search TODO
struct Sequences {
    const DiGraph *graph{nullptr};
    unordered_map<int64_t, std::vector<std::vector<int64_t>>> head2seqs;

    std::map<int, std::vector<std::vector<int64_t>>> search_in(const std::vector<int64_t> &nodes,
                                                               bool quick_return = true) const;
};

// head/tail of DiGraph node(road)
using Endpoint = std::array<double, 2>;
struct Endpoints {
    const DiGraph *graph{nullptr};
    bool is_wgs84 = true;
    unordered_map<int64_t, std::tuple<Endpoint, Endpoint>> endpoints;  // (head, tail)
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

    bool valid() const { return graph && !nodes.empty(); }

    // idx, t
    std::tuple<int, double> along(double offset) const;
    std::optional<Path> slice(double start_offset, double end_offset) const;
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

// road
struct Node {
    double length{1.0};
};

// link
struct Edge {
    //
};

struct DiGraph {
    DiGraph() = default;
    Node &add_node(const std::string &id, double length = 1.0);
    Edge &add_edge(const std::string &node0, const std::string &node1);

    std::vector<std::string> prevs(const std::string &id) const { return __nexts(id, prevs_); }
    std::vector<std::string> nexts(const std::string &id) const { return __nexts(id, nexts_); }
    const std::unordered_map<std::string, std::unordered_set<std::string>> sibs_under_next() const;
    const std::unordered_map<std::string, std::unordered_set<std::string>> sibs_under_prev() const;

    const std::unordered_map<std::string, Node *> &nodes() const { return cache().nodes; }
    const std::unordered_map<std::tuple<std::string, std::string>, Edge *> &edges() const { return cache().edges; }

    // encode sinks/bindings/sequences/endpoints
    // it's not const, because we need to index road ids
    Sinks encode_sinks(const std::unordered_set<std::string> &nodes,
                       const std::unordered_map<std::string, std::unordered_set<std::string>> &links);
    Bindings encode_bindings(const std::unordered_map<std::string, std::vector<Binding>> &bindings);
    Sequences encode_sequences(const std::vector<std::vector<std::string>> &sequences);
    Endpoints encode_endpoints(const std::unordered_map<std::string, std::tuple<Endpoint, Endpoint>> &endpoints,
                               bool is_wgs84 = true);

    std::optional<UbodtRecord> encode_ubodt(const std::string &source_road, const std::string &target_road,
                                            const std::string &source_next, const std::string &target_prev,
                                            double cost) const;

    // str -> id, len
    std::optional<int64_t> __node_id(const std::string &node) const;
    std::optional<std::tuple<int64_t, double>> __node_length(const std::string &node) const;

    std::string __node_id(int64_t node) const { return indexer_.id(node); }
    std::vector<std::string> __node_ids(const std::vector<int64_t> &nodes) const;
    double length(int64_t node) const;

    std::optional<Path> shortest_path(const std::string &source,            //
                                      const std::string &target,            //
                                      double cutoff,                        //
                                      std::optional<double> source_offset,  //
                                      std::optional<double> target_offset,  //
                                      const Sinks *sinks = nullptr,         //
                                      const Endpoints *endpoints = nullptr) const;

    std::optional<ZigzagPath> shortest_zigzag_path(const std::string &source,                 //
                                                   const std::optional<std::string> &target,  //
                                                   double cutoff,                             //
                                                   int direction = 0,                         //
                                                   ZigzagPathGenerator *generator = nullptr) const;

    ShortestPathGenerator shortest_paths(const std::string &start,           //
                                         double cutoff,                      //
                                         std::optional<double> offset = {},  //
                                         bool reverse = false,               //
                                         const Sinks *sinks = nullptr) const;

    std::vector<Path> all_paths_from(const std::string &source, double cutoff, std::optional<double> offset = {},
                                     const Sinks *sinks = nullptr) const;

    std::vector<Path> all_paths_to(const std::string &target, double cutoff, std::optional<double> offset = {},
                                   const Sinks *sinks = nullptr) const;
    std::vector<Path> all_paths(const std::string &source,            //
                                const std::string &target,            //
                                double cutoff,                        //
                                std::optional<double> source_offset,  //
                                std::optional<double> target_offset,  //
                                const Sinks *sinks = nullptr) const;

    std::tuple<std::optional<Path>, std::optional<Path>> shortest_path_to_bindings(
        const std::string &source,          //
        double cutoff,                      //
        const Bindings &bindings,           //
        std::optional<double> offset = {},  //
        int direction = 0,                  // 0 -> forwards/backwards, 1->forwards, -1:backwards
        const Sinks *sinks = nullptr) const;
    std::tuple<std::optional<double>, std::optional<double>> distance_to_bindings(const std::string &source,          //
                                                                                  double cutoff,                      //
                                                                                  const Bindings &bindings,           //
                                                                                  std::optional<double> offset = {},  //
                                                                                  int direction = 0,                  //
                                                                                  const Sinks *sinks = nullptr) const;

    std::tuple<std::vector<Path>, std::vector<Path>> all_paths_to_bindings(const std::string &source,          //
                                                                           double cutoff,                      //
                                                                           const Bindings &bindings,           //
                                                                           std::optional<double> offset = {},  //
                                                                           int direction = 0,                  //
                                                                           const Sinks *sinks = nullptr,       //
                                                                           bool with_endings = false) const;

    std::vector<UbodtRecord> build_ubodt(int64_t source, double thresh) const;
    std::vector<UbodtRecord> build_ubodt(double thresh, int pool_size = 1, int nodes_thresh = 1000) const;

    void freeze() { freezed_ = true; }
    void build() const;
    void reset() const;

    Indexer &indexer() { return indexer_; }
    const Indexer &indexer() const { return indexer_; }

  private:
    bool freezed_{false};
    // nodes, edges
    // cannot use ankerl unordered_map, because it's not memory stable
    std::unordered_map<int64_t, Node> nodes_;
    std::unordered_map<std::tuple<int64_t, int64_t>, Edge> edges_;
    // topo
    unordered_map<int64_t, double> lengths_;
    unordered_map<int64_t, unordered_set<int64_t>> nexts_, prevs_;
    // index road id
    mutable Indexer indexer_;
    struct Cache {
        std::unordered_map<std::string, Node *> nodes;
        std::unordered_map<std::tuple<std::string, std::string>, Edge *> edges;
        unordered_map<int64_t, unordered_set<int64_t>> sibs_under_prev, sibs_under_next;
    };
    mutable std::optional<Cache> cache_;
    Cache &cache() const;

    std::vector<std::string> __nexts(const std::string &id,
                                     const unordered_map<int64_t, unordered_set<int64_t>> &jumps) const;

    std::optional<Path> __dijkstra(int64_t source, int64_t target, double cutoff, const Sinks *sinks = nullptr) const;
    std::optional<Path> __astar(int64_t source, int64_t target, double cutoff, const Endpoints &endpoints,
                                const Sinks *sinks = nullptr) const;

    std::optional<ZigzagPath> __shortest_zigzag_path(int64_t source, std::optional<int64_t> target, double cutoff,
                                                     int direction = 0, ZigzagPathGenerator *generator = nullptr) const;

    std::optional<Path> __shortest_path_to_bindings(int64_t source, std::optional<double> source_offset,
                                                    double source_length,
                                                    double cutoff,                 //
                                                    const Bindings &bindings,      //
                                                    const Sinks *sinks = nullptr,  //
                                                    bool reverse = false) const;

    std::vector<Path> __all_paths(int64_t source, double cutoff, std::optional<double> offset,
                                  const unordered_map<int64_t, double> &lengths,
                                  const unordered_map<int64_t, unordered_set<int64_t>> &jumps,
                                  const Sinks *sinks = nullptr) const;

    std::vector<Path> __all_paths(int64_t source, int64_t target, double cutoff, const Sinks *sinks = nullptr) const;

    std::vector<Path> __all_path_to_bindings__(int64_t source,                       //
                                               std::optional<double> source_offset,  //
                                               double source_length,                 //
                                               double cutoff,                        //
                                               const Bindings &bindings,             //
                                               const Sinks *sinks,                   //
                                               bool reverse) const;

    std::vector<Path> __all_path_to_bindings(int64_t source,                       //
                                             std::optional<double> source_offset,  //
                                             double source_length,                 //
                                             double cutoff,                        //
                                             const Bindings &bindings,             //
                                             const Sinks *sinks,                   //
                                             bool reverse,                         //
                                             bool with_endings) const;
};

}  // namespace cubao
