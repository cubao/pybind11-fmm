#pragma once

#include "3rdparty/packedrtree.hpp"
#include "graph.hpp"
#include "types.hpp"

namespace cubao {

struct ProjectedPoint {
    int64_t edge_id;
    double offset;
    double distance;
    Eigen::Vector2d point;
};

struct Network {
    using RTree = FlatGeobuf::PackedRTree;
    DiGraph graph;
    unordered_map<int64_t, Polyline> geometries;

    void add_edge(int64_t edge_id, const Eigen::Ref<const RowVectors> &coords, bool is_wgs84 = true) {
        geometries.emplace(edge_id, Polyline(coords, is_wgs84));
    }

    const Polyline &geometry(int64_t edge_id) const { return geometries.at(edge_id); }

    std::vector<ProjectedPoint> query_radius(const Eigen::Vector2d &pt, double radius) const;

    Path shortest_path(int64_t from_edge, double from_offset, int64_t to_edge, double to_offset) const;

  private:
    void build_spatial_index() const;
    mutable std::optional<RTree> spatial_index_;
    mutable std::vector<int64_t> edge_ids_;  // Mapping: rtree index → edge ID
};

}  // namespace cubao
