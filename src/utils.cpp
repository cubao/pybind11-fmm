#include "utils.hpp"

#include <queue>

namespace cubao {

inline Eigen::VectorXi douglas_simplify_iter(const Eigen::Ref<const RowVectors> &coords, const double epsilon) {
    Eigen::VectorXi to_keep(coords.rows());
    to_keep.setZero();
    std::queue<std::pair<int, int>> q;
    q.push({0, to_keep.size() - 1});
    double eps2 = epsilon * epsilon;
    while (!q.empty()) {
        int i = q.front().first;
        int j = q.front().second;
        q.pop();
        to_keep[i] = to_keep[j] = 1;
        if (j - i <= 1) {
            continue;
        }
        LineSegment line(coords.row(i), coords.row(j));
        double max_dist2 = 0.0;
        int max_index = i;
        for (int k = i + 1; k < j; ++k) {
            double dist2 = line.dist2(coords.row(k));
            if (dist2 > max_dist2) {
                max_dist2 = dist2;
                max_index = k;
            }
        }
        if (max_dist2 <= eps2) {
            continue;
        }
        q.push({i, max_index});
        q.push({max_index, j});
    }
    return to_keep;
}

inline Eigen::VectorXi douglas_simplify_mask(const Eigen::Ref<const RowVectors> &coords,
                                             double epsilon,  //
                                             bool is_wgs84) {
    if (is_wgs84) {
        return douglas_simplify_mask(wgs84_to_xy(coords), epsilon, false);
    }
    return douglas_simplify_iter(coords, epsilon);
}

inline RowVectors select_by_mask(const Eigen::Ref<const RowVectors> &coords,
                                 const Eigen::Ref<const Eigen::VectorXi> &mask) {
    RowVectors ret(mask.sum(), coords.cols());
    int N = mask.size();
    for (int i = 0, k = 0; i < N; ++i) {
        if (mask[i]) {
            ret.row(k++) = coords.row(i);
        }
    }
    return ret;
}

RowVectors douglas_simplify(const Eigen::Ref<const RowVectors> &coords, double epsilon, bool is_wgs84) {
    if (coords.rows() <= 2) {
        return coords;
    }
    return select_by_mask(coords, douglas_simplify_mask(coords, epsilon, is_wgs84));
}

std::string encode_polyline(const Eigen::Ref<const RowVectors> &coords) { return "TODO"; }

RowVectors decode_polyline(const std::string &encoded) {
    // https://valhalla.github.io/valhalla/decoding/#c-11
    constexpr double kPolylinePrecision = 1E6;
    constexpr double kInvPolylinePrecision = 1.0 / kPolylinePrecision;
    size_t i = 0;  // what byte are we looking at

    // Handy lambda to turn a few bytes of an encoded string into an integer
    auto deserialize = [&encoded, &i](const int previous) {
        // Grab each 5 bits and mask it in where it belongs using the shift
        int byte, shift = 0, result = 0;
        do {
            byte = static_cast<int>(encoded[i++]) - 63;
            result |= (byte & 0x1f) << shift;
            shift += 5;
        } while (byte >= 0x20);
        // Undo the left shift from above or the bit flipping and add to previous
        // since its an offset
        return previous + (result & 1 ? ~(result >> 1) : (result >> 1));
    };

    // Iterate over all characters in the encoded string
    std::vector<std::array<double, 2>> shape;
    int last_lon = 0, last_lat = 0;
    while (i < encoded.length()) {
        // Decode the coordinates, lat first for some reason
        int lat = deserialize(last_lat);
        int lon = deserialize(last_lon);

        // Shift the decimal point 5 places to the left
        shape.push_back({static_cast<double>(lon) * kInvPolylinePrecision,  //
                         static_cast<double>(lat) * kInvPolylinePrecision});

        // Remember the last one we encountered
        last_lon = lon;
        last_lat = lat;
    }
    return Eigen::Map<const RowVectors>(&shape[0][0], shape.size(), 2);
}

}  // namespace cubao
