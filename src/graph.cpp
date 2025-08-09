#include "graph.hpp"

#include "heap.hpp"
#include "utils.hpp"

namespace cubao {
//

// round to 10cm
#define __ROUND__(x) ROUND(x, 10)
// #define __ROUND__(x) x

// void __round__(Path &path) { path.dist }

// void round(Path &r)
// {
//     r.dist = ROUND(r.dist, *round_scale_);
//     if (r.start_offset) {
//         r.start_offset = ROUND(*r.start_offset, *round_scale_);
//     }
//     if (r.end_offset) {
//         r.end_offset = ROUND(*r.end_offset, *round_scale_);
//     }
// }

static void single_source_dijkstra(unordered_map<int64_t, int64_t> &pmap,                        //
                                   unordered_map<int64_t, double> &dmap,                         //
                                   int64_t start, double cutoff,                                 //
                                   const unordered_map<int64_t, unordered_set<int64_t>> &jumps,  //
                                   const unordered_map<int64_t, double> &lengths,                //
                                   const Sinks *sinks = nullptr,                                 //
                                   double init_offset = 0.0) {
    // https://github.com/cyang-kth/fmm/blob/5cccc608903877b62969e41a58b60197a37a5c01/src/network/network_graph.cpp#L234-L274
    // https://github.com/cubao/nano-fmm/blob/37d2979503f03d0a2759fc5f110e2b812d963014/src/nano_fmm/network.cpp#L449C67-L449C72
    /*
    if (cutoff < init_offset) {
        return;
    }
    auto itr = jumps.find(start);
    if (itr == jumps.end()) {
        return;
    }
    Heap Q;
    Q.push(start, 0.0);
    if (!sinks || !sinks->nodes.count(start)) {
        for (auto next : itr->second) {
            Q.push(next, init_offset);
            pmap.insert({next, start});
            dmap.insert({next, init_offset});
        }
    }
    while (!Q.empty()) {
        HeapNode node = Q.top();
        Q.pop();
        if (node.value > cutoff) {
            break;
        }
        auto u = node.index;
        if (sinks && sinks->nodes.count(u)) {
            continue;
        }
        auto itr = jumps.find(u);
        if (itr == jumps.end()) {
            continue;
        }
        double u_cost = lengths.at(u);
        for (auto v : itr->second) {
            auto c = node.value + u_cost;
            auto iter = dmap.find(v);
            if (iter != dmap.end()) {
                if (c < iter->second) {
                    pmap[v] = u;
                    dmap[v] = c;
                    if (Q.contain_node(v)) {
                        Q.decrease_key(v, c);
                    } else {
                        Q.push(v, c);
                    }
                }
            } else {
                if (c <= cutoff) {
                    pmap.insert({v, u});
                    dmap.insert({v, c});
                    Q.push(v, c);
                }
            }
        }
    }
    dmap.erase(start);
    */
}

void Bindings::sort() {
    // todo, sort bindings
}

std::map<int, std::vector<std::vector<int64_t>>> Sequences::search_in(const std::vector<int64_t> &nodes,
                                                                      bool quick_return) const {
    std::map<int, std::vector<std::vector<int64_t>>> ret;
    for (int i = 0, N = nodes.size(); i < N; ++i) {
        auto itr = head2seqs.find(nodes[i]);
        if (itr == head2seqs.end()) {
            continue;
        }
        for (auto &c : itr->second) {
            if (c.size() > N - i) {
                continue;
            }
            if (std::equal(c.begin(), c.end(), &nodes[i])) {
                ret[i].push_back(c);
                if (quick_return) {
                    return ret;
                }
            }
        }
    }
    return ret;
}

// idx, t
std::tuple<int, double> Path::along(double offset) const {
    auto &self = *this;
    if (offset <= 0) {
        int idx = 0;
        auto nid = self.nodes.at(idx);
        return std::make_tuple(idx, self.start_offset.value_or(self.graph->length(nid)));
    } else if (offset >= self.dist) {
        int idx = self.nodes.size() - 1;
        return std::make_tuple(idx, self.end_offset.value_or(0.0));
    }
    if (self.start_offset) {
        double remain = std::max(0.0, self.graph->length(self.nodes.front()) - *self.start_offset);
        if (offset <= remain) {
            return std::make_tuple(0, *self.start_offset + offset);
        }
        offset -= remain;
    }
    for (int i = 1; i < self.nodes.size(); ++i) {
        auto nid = self.nodes.at(i);
        double length = self.graph->length(nid);
        if (offset <= length) {
            return std::make_tuple(i, offset);
        }
        offset -= length;
    }
    int idx = self.nodes.size() - 1;
    return std::make_tuple(idx, self.end_offset.value_or(0.0));
}

using State = ZigzagPathGenerator::State;
std::optional<ZigzagPath> ZigzagPathGenerator::Path(const State &state, const int64_t source,  //
                                                    const DiGraph *self,                       //
                                                    const unordered_map<State, State> &pmap,
                                                    const unordered_map<State, double> &dmap) {
    std::vector<State> states;
    int64_t target = std::get<0>(state);
    int dir = -std::get<1>(state);
    double dist = dmap.at(state);
    auto cursor = state;
    while (true) {
        auto prev = pmap.find(cursor);
        if (prev == pmap.end()) {
            // assert cursor at source
            if (std::get<0>(cursor) != source) {
                return {};
            }
            states.push_back({source, -std::get<1>(cursor)});
            break;
        }
        cursor = prev->second;
        states.push_back(cursor);
    }
    std::reverse(states.begin(), states.end());
    size_t N = states.size();
    if (N % 2 != 0) {
        return {};
    }
    auto nodes = std::vector<int64_t>{};
    auto dirs = std::vector<int>{};
    for (size_t i = 0; i < N; i += 2) {
        if (std::get<0>(states[i]) != std::get<0>(states[i + 1])) {
            return {};
        }
        nodes.push_back(std::get<0>(states[i]));
        dirs.push_back(std::get<1>(states[i]) < std::get<1>(states[i + 1]) ? 1 : -1);
    }
    nodes.push_back(target);
    dirs.push_back(dir);
    return ZigzagPath(self, dist, nodes, dirs);
}

Node &DiGraph::add_node(const std::string &id, double length) {
    if (freezed_) {
        throw std::logic_error("DiGraph already freezed!");
    }
    reset();
    length = __ROUND__(length);
    auto idx = indexer_.id(id);
    auto &node = nodes_[idx];
    node.length = length;
    lengths_[idx] = length;
    return node;
}

Edge &DiGraph::add_edge(const std::string &node0, const std::string &node1) {
    if (freezed_) {
        throw std::logic_error("DiGraph already freezed!");
    }
    reset();
    auto idx0 = indexer_.id(node0);
    auto idx1 = indexer_.id(node1);
    nexts_[idx0].insert(idx1);
    prevs_[idx1].insert(idx0);
    lengths_[idx0] = nodes_[idx0].length;
    lengths_[idx1] = nodes_[idx1].length;
    auto &edge = edges_[std::make_tuple(idx0, idx1)];
    return edge;
}

const std::unordered_map<std::string, std::unordered_set<std::string>> DiGraph::sibs_under_next() const {
    auto ret = std::unordered_map<std::string, std::unordered_set<std::string>>{};
    for (auto &kv : cache().sibs_under_next) {
        auto &sibs = ret[indexer_.id(kv.first)];
        for (auto s : kv.second) {
            sibs.insert(indexer_.id(s));
        }
    }
    return ret;
}

const std::unordered_map<std::string, std::unordered_set<std::string>> DiGraph::sibs_under_prev() const {
    auto ret = std::unordered_map<std::string, std::unordered_set<std::string>>{};
    for (auto &kv : cache().sibs_under_prev) {
        auto &sibs = ret[indexer_.id(kv.first)];
        for (auto s : kv.second) {
            sibs.insert(indexer_.id(s));
        }
    }
    return ret;
}

Sinks DiGraph::encode_sinks(const std::unordered_set<std::string> &nodes,
                            const std::unordered_map<std::string, std::unordered_set<std::string>> &links) {
    Sinks ret;
    ret.graph = this;
    for (auto &n : nodes) {
        ret.nodes.insert(indexer_.id(n));
    }
    return ret;
}

Bindings DiGraph::encode_bindings(const std::unordered_map<std::string, std::vector<Binding>> &bindings) {
    Bindings ret;
    ret.graph = this;
    for (auto &pair : bindings) {
        auto [itr, _] = ret.node2bindings.emplace(indexer_.id(pair.first), pair.second);
        std::sort(itr->second.begin(), itr->second.end(), [](const auto &a, const auto &b) {
            return std::tie(std::get<0>(a), std::get<1>(a)) < std::tie(std::get<0>(b), std::get<1>(b));
        });
    }
    return ret;
}

Sequences DiGraph::encode_sequences(const std::vector<std::vector<std::string>> &sequences) {
    Sequences ret;
    ret.graph = this;
    for (auto &seq : sequences) {
        if (seq.empty()) {
            continue;
        }
        std::vector<int64_t> nodes;
        nodes.reserve(seq.size());
        for (auto s : seq) {
            nodes.push_back(indexer_.id(s));
        }
        ret.head2seqs[nodes[0]].push_back(nodes);
    }
    return ret;
}

Endpoints DiGraph::encode_endpoints(const std::unordered_map<std::string, std::tuple<Endpoint, Endpoint>> &endpoints,
                                    bool is_wgs84) {
    Endpoints ret;
    ret.graph = this;
    ret.is_wgs84 = is_wgs84;
    for (auto &pair : endpoints) {
        ret.endpoints.emplace(indexer_.id(pair.first), pair.second);
    }
    return ret;
}

std::optional<UbodtRecord> DiGraph::encode_ubodt(const std::string &source_road, const std::string &target_road,
                                                 const std::string &source_next, const std::string &target_prev,
                                                 double cost) const {
    auto sroad = indexer_.get_id(source_road);
    if (!sroad) {
        return {};
    }
    auto troad = indexer_.get_id(target_road);
    if (!troad) {
        return {};
    }
    auto snext = indexer_.get_id(source_next);
    if (!snext) {
        return {};
    }
    auto tprev = indexer_.get_id(target_prev);
    if (!tprev) {
        return {};
    }
    return UbodtRecord(*sroad, *troad, *snext, *tprev, cost);
}

std::optional<int64_t> DiGraph::__node_id(const std::string &node) const { return indexer_.get_id(node); }

std::optional<std::tuple<int64_t, double>> DiGraph::__node_length(const std::string &node) const {
    auto nid = __node_id(node);
    if (!nid) {
        return {};
    }
    auto len = lengths_.find(*nid);
    if (len == lengths_.end()) {
        return {};
    }
    return std::make_tuple(*nid, len->second);
}

std::vector<std::string> DiGraph::__node_ids(const std::vector<int64_t> &nodes) const {
    std::vector<std::string> ids;
    ids.reserve(nodes.size());
    for (auto node : nodes) {
        ids.push_back(indexer_.id(node));
    }
    return ids;
}
double DiGraph::length(int64_t node) const { return lengths_.at(node); }

std::optional<Path> DiGraph::shortest_path(const std::string &source,            //
                                           const std::string &target,            //
                                           double cutoff,                        //
                                           std::optional<double> source_offset,  //
                                           std::optional<double> target_offset,  //
                                           const Sinks *sinks,                   //
                                           const Endpoints *endpoints) const {
    if (cutoff < 0) {
        return {};
    }
    if (sinks && sinks->graph != this) {
        sinks = nullptr;
    }
    auto src_idx = indexer_.get_id(source);
    if (!src_idx) {
        return {};
    }
    auto src_length = lengths_.find(*src_idx);
    if (src_length == lengths_.end()) {
        return {};
    }
    auto dst_idx = indexer_.get_id(target);
    if (!dst_idx) {
        return {};
    }
    auto dst_length = lengths_.find(*dst_idx);
    if (dst_length == lengths_.end()) {
        return {};
    }
    if (source_offset) {
        source_offset = CLIP(0.0, *source_offset, src_length->second);
    }
    if (target_offset) {
        target_offset = CLIP(0.0, *target_offset, dst_length->second);
    }
    std::optional<Path> path;
    if (*src_idx == *dst_idx) {
        if (!source_offset && !target_offset) {
            path = Path(this, 0.0, std::vector<int64_t>{*src_idx});
        } else if (source_offset && target_offset) {
            double dist = *target_offset - *source_offset;
            if (dist < 0 || dist > cutoff) {
                return {};
            }
            path = Path(this, dist, std::vector<int64_t>{*src_idx}, source_offset, target_offset);
        } else {
            return {};
        }
    } else {
        double delta = 0.0;
        if (source_offset) {
            delta += src_length->second - *source_offset;
        }
        if (target_offset) {
            delta += *target_offset;
        }
        path = endpoints ? __astar(*src_idx, *dst_idx, cutoff - delta, *endpoints, sinks)
                         : __dijkstra(*src_idx, *dst_idx, cutoff - delta, sinks);
        if (path) {
            path->dist += delta;
            path->start_offset = source_offset;
            path->end_offset = target_offset;
        }
    }
    // if (path && round_scale_) {
    //     path->round(*round_scale_);
    // }
    return path;
}

std::optional<ZigzagPath> DiGraph::shortest_zigzag_path(const std::string &source,                 //
                                                        const std::optional<std::string> &target,  //
                                                        double cutoff,                             //
                                                        int direction,                             //
                                                        ZigzagPathGenerator *generator) const {
    if (cutoff < 0) {
        return {};
    }
    bool one_and_only = bool(target) ^ bool(generator);
    if (!one_and_only) {
        return {};
    }
    auto src_idx = indexer_.get_id(source);
    if (!src_idx) {
        return {};
    }
    std::optional<int64_t> dst_idx;
    if (target) {
        dst_idx = indexer_.get_id(*target);
        if (!dst_idx) {
            return {};
        }
    }
    auto path = __shortest_zigzag_path(*src_idx, dst_idx, cutoff, direction, generator);
    // if (path && round_scale_) {
    //     path->round(*round_scale_);
    // }
    return path;
}

std::vector<Path> DiGraph::all_paths_from(const std::string &source, double cutoff,  //
                                          std::optional<double> offset,              //
                                          const Sinks *sinks) const {
    if (cutoff < 0) {
        return {};
    }
    if (sinks && sinks->graph != this) {
        sinks = nullptr;
    }
    auto src_idx = indexer_.get_id(source);
    if (!src_idx) {
        return {};
    }
    auto paths = __all_paths(*src_idx, cutoff, offset, lengths_, nexts_, sinks);
    // if (round_scale_) {
    //     for (auto &p : paths) {
    //         p.round(*round_scale_);
    //     }
    // }
    return paths;
}

std::vector<Path> DiGraph::all_paths_to(const std::string &target, double cutoff,  //
                                        std::optional<double> offset,              //
                                        const Sinks *sinks) const {
    if (cutoff < 0) {
        return {};
    }
    if (sinks && sinks->graph != this) {
        sinks = nullptr;
    }
    auto dst_idx = indexer_.get_id(target);
    if (!dst_idx) {
        return {};
    }
    auto length = lengths_.find(*dst_idx);
    if (length == lengths_.end()) {
        return {};
    }
    if (offset) {
        offset = CLIP(0.0, *offset, length->second);
        offset = length->second - *offset;
    }
    auto paths = __all_paths(*dst_idx, cutoff, offset, lengths_, prevs_, sinks);
    for (auto &p : paths) {
        if (p.start_offset) {
            p.start_offset = lengths_.at(p.nodes.front()) - *p.start_offset;
        }
        if (p.end_offset) {
            p.end_offset = lengths_.at(p.nodes.back()) - *p.end_offset;
        }
        std::reverse(p.nodes.begin(), p.nodes.end());
        std::swap(p.start_offset, p.end_offset);
    }
    // if (round_scale_) {
    //     for (auto &p : paths) {
    //         p.round(*round_scale_);
    //     }
    // }
    return paths;
}

std::vector<Path> DiGraph::all_paths(const std::string &source,            //
                                     const std::string &target,            //
                                     double cutoff,                        //
                                     std::optional<double> source_offset,  //
                                     std::optional<double> target_offset,  //
                                     const Sinks *sinks) const {
    if (cutoff < 0) {
        return {};
    }
    if (sinks && sinks->graph != this) {
        sinks = nullptr;
    }
    auto src_idx = indexer_.get_id(source);
    if (!src_idx) {
        return {};
    }
    auto src_length = lengths_.find(*src_idx);
    if (src_length == lengths_.end()) {
        return {};
    }
    auto dst_idx = indexer_.get_id(target);
    if (!dst_idx) {
        return {};
    }
    auto dst_length = lengths_.find(*dst_idx);
    if (dst_length == lengths_.end()) {
        return {};
    }
    if (source_offset) {
        source_offset = CLIP(0.0, *source_offset, src_length->second);
    }
    if (target_offset) {
        target_offset = CLIP(0.0, *target_offset, dst_length->second);
    }
    std::vector<Path> paths;
    if (*src_idx == *dst_idx) {
        if (!source_offset || !target_offset) {
            return {};
        }
        if (*target_offset - *source_offset > cutoff) {
            return {};
        }
        double dist = *target_offset - *source_offset;
        if (dist <= 0) {
            return {};
        }
        paths.emplace_back(this, dist, std::vector<int64_t>{*src_idx}, source_offset, target_offset);
    } else {
        double delta = 0.0;
        if (source_offset) {
            delta += src_length->second - *source_offset;
        }
        if (target_offset) {
            delta += *target_offset;
        }
        cutoff -= delta;
        paths = __all_paths(*src_idx, *dst_idx, cutoff, sinks);
        for (auto &p : paths) {
            p.dist += delta;
            p.start_offset = source_offset;
            p.end_offset = target_offset;
        }
    }
    // if (round_scale_) {
    //     for (auto &p : paths) {
    //         p.round(*round_scale_);
    //     }
    // }
    return paths;
}

std::tuple<std::optional<Path>, std::optional<Path>> DiGraph::shortest_path_to_bindings(
    const std::string &source,     //
    double cutoff,                 //
    const Bindings &bindings,      //
    std::optional<double> offset,  //
    int direction,                 // 0 -> forwards/backwards, 1->forwards, -1:backwards
    const Sinks *sinks) const {
    if (bindings.graph != this) {
        return {};
    }
    if (cutoff < 0) {
        return {};
    }
    if (sinks && sinks->graph != this) {
        sinks = nullptr;
    }
    auto src_idx = indexer_.get_id(source);
    if (!src_idx) {
        return {};
    }
    auto length = lengths_.find(*src_idx);
    if (length == lengths_.end()) {
        return {};
    }
    std::optional<Path> forward_path;
    if (direction >= 0) {
        forward_path = __shortest_path_to_bindings(*src_idx, offset, length->second, cutoff, bindings, sinks);
    }
    std::optional<Path> backward_path;
    if (direction <= 0) {
        backward_path = __shortest_path_to_bindings(*src_idx, offset, length->second, cutoff, bindings, sinks, true);
    }
    // if (round_scale_) {
    //     if (forward_path) {
    //         forward_path->round(*round_scale_);
    //     }
    //     if (backward_path) {
    //         backward_path->round(*round_scale_);
    //     }
    // }
    return std::make_tuple(backward_path, forward_path);
}

DiGraph::Cache &DiGraph::cache() const {
    if (cache_) {
        return *cache_;
    }
    auto cache = Cache();
    for (auto &pair : nodes_) {
        cache.nodes.emplace(indexer_.id(pair.first), const_cast<Node *>(&pair.second));
    }
    for (auto &pair : edges_) {
        cache.edges.emplace(std::make_tuple(indexer_.id(std::get<0>(pair.first)), indexer_.id(std::get<1>(pair.first))),
                            const_cast<Edge *>(&pair.second));
    }
    {
        auto &sibs = cache.sibs_under_next;
        for (auto &kv : nexts_) {
            if (kv.second.size() > 1) {
                for (auto pid : kv.second) {
                    sibs[pid].insert(kv.second.begin(), kv.second.end());
                }
            }
        }
        for (auto &kv : sibs) {
            kv.second.erase(kv.first);
        }
    }
    {
        auto &sibs = cache.sibs_under_prev;
        for (auto &kv : prevs_) {
            if (kv.second.size() > 1) {
                for (auto nid : kv.second) {
                    sibs[nid].insert(kv.second.begin(), kv.second.end());
                }
            }
        }
        for (auto &kv : sibs) {
            kv.second.erase(kv.first);
        }
    }
    cache_ = std::move(cache);
    return *cache_;
}

std::optional<Path> DiGraph::__dijkstra(int64_t source, int64_t target, double cutoff, const Sinks *sinks) const {
    // TODO: implement dijkstra algorithm
    return {};
}

std::optional<Path> DiGraph::__astar(int64_t source, int64_t target, double cutoff,  //
                                     const Endpoints &endpoints,                     //
                                     const Sinks *sinks) const {
    // TODO: implement A* algorithm
    return {};
}

std::optional<ZigzagPath> DiGraph::__shortest_zigzag_path(int64_t source, std::optional<int64_t> target,
                                                          double cutoff,  //
                                                          int direction,  //
                                                          ZigzagPathGenerator *generator) const {
    // TODO: implement zigzag path finding
    return {};
}

std::optional<Path> DiGraph::__shortest_path_to_bindings(int64_t source, std::optional<double> source_offset,
                                                         double source_length, double cutoff,  //
                                                         const Bindings &bindings,             //
                                                         const Sinks *sinks, bool reverse) const {
    // TODO: implement shortest path to bindings
    return {};
}

std::vector<Path> DiGraph::__all_paths(int64_t source, double cutoff, std::optional<double> offset,
                                       const unordered_map<int64_t, double> &lengths,
                                       const unordered_map<int64_t, unordered_set<int64_t>> &jumps,
                                       const Sinks *sinks) const {
    // TODO: implement all paths from source
    return {};
}

std::vector<Path> DiGraph::__all_paths(int64_t source, int64_t target, double cutoff, const Sinks *sinks) const {
    // TODO: implement all paths between source and target
    return {};
}

std::vector<Path> DiGraph::__all_path_to_bindings__(int64_t source, std::optional<double> source_offset,
                                                    double source_length, double cutoff, const Bindings &bindings,
                                                    const Sinks *sinks, bool reverse) const {
    // TODO: implement all paths to bindings
    return {};
}

std::vector<Path> DiGraph::__all_path_to_bindings(int64_t source, std::optional<double> source_offset,
                                                  double source_length, double cutoff, const Bindings &bindings,
                                                  const Sinks *sinks, bool reverse, bool with_endings) const {
    // TODO: implement all paths to bindings with endings
    return {};
}

std::vector<UbodtRecord> DiGraph::build_ubodt(int64_t source, double thresh) const {
    // TODO: implement UBODT construction for single source
    return {};
}

std::vector<UbodtRecord> DiGraph::build_ubodt(double thresh, int pool_size, int nodes_thresh) const {
    // TODO: implement UBODT construction for all sources
    return {};
}

void DiGraph::build() const {
    // TODO: build graph indices
}

void DiGraph::reset() const { cache_.reset(); }

}  // namespace cubao
