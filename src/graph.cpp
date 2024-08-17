#include "graph.hpp"

namespace cubao {
//

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

}  // namespace cubao
