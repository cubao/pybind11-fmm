#include "network.hpp"

#include <algorithm>

namespace cubao {

void Network::build_spatial_index() const {
    if (spatial_index_) {
        return;  // Already built
    }

    // Build bounding boxes for all edges
    std::vector<FlatGeobuf::NodeItem> node_items;
    std::vector<int64_t> edge_ids;

    for (const auto& [edge_id, polyline] : geometries) {
        const auto& bbox = polyline.bbox();
        FlatGeobuf::NodeItem item{bbox[0],  // minX
                                  bbox[1],  // minY
                                  bbox[2],  // maxX
                                  bbox[3],  // maxY
                                  static_cast<uint64_t>(node_items.size())};
        node_items.push_back(item);
        edge_ids.push_back(edge_id);
    }

    if (node_items.empty()) {
        return;  // No edges to index
    }

    // Calculate extent
    FlatGeobuf::NodeItem extent = FlatGeobuf::calcExtent(node_items);

    // Build the RTree
    spatial_index_ = RTree(node_items, extent);

    // Store edge_ids mapping (note: this is a mutable member to support lazy init)
    // We use const_cast since this is a lazy initialization pattern
    const_cast<std::vector<int64_t>&>(edge_ids_).swap(edge_ids);
}

std::vector<ProjectedPoint> Network::query_radius(const Eigen::Vector2d& pt, double radius) const {
    // Lazy build spatial index
    build_spatial_index();

    if (!spatial_index_) {
        return {};  // No edges indexed
    }

    // Create query bounding box
    double minX = pt.x() - radius;
    double minY = pt.y() - radius;
    double maxX = pt.x() + radius;
    double maxY = pt.y() + radius;

    // Query R-tree
    auto search_results = spatial_index_->search(minX, minY, maxX, maxY);

    // Project query point onto each candidate edge
    std::vector<ProjectedPoint> projected_points;
    projected_points.reserve(search_results.size());

    for (const auto& result : search_results) {
        int64_t edge_id = edge_ids_[result.index];
        const auto& polyline = geometries.at(edge_id);

        // Find nearest point on polyline
        auto [proj_pt, dist, seg_idx, t] = polyline.nearest(pt);

        // Only include if within radius
        if (dist <= radius) {
            // Calculate offset along polyline
            double offset = polyline.offset(seg_idx, t);

            projected_points.push_back({edge_id, offset, dist, proj_pt});
        }
    }

    return projected_points;
}

Path Network::shortest_path(int64_t from_edge, double from_offset, int64_t to_edge, double to_offset) const {
    // TODO: Implement shortest path with edge offsets
    // This requires:
    // 1. Find nodes at from_edge and to_edge
    // 2. Run Dijkstra on graph
    // 3. Construct path with start/end offsets

    // For now, return empty path
    // This will be implemented after we understand the graph structure better
    return Path();
}

}  // namespace cubao
