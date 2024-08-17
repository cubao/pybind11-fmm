#include "graph.hpp"

namespace cubao {
//

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

}  // namespace cubao
