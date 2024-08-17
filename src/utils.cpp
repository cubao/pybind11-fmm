#include "utils.hpp"

#include <queue>

namespace cubao {

inline Eigen::VectorXi douglas_simplify_iter(const Eigen::Ref<const RowVectors> &coords, const double epsilon) {
    Eigen::VectorXi to_keep(coords.rows());
    to_keep.setZero();
    std::queue<std::pair<int, int>> q;
    q.push({0, to_keep.size() - 1});
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
        if (max_dist2 <= epsilon * epsilon) {
            continue;
        }
        q.push({i, max_index});
        q.push({max_index, j});
    }
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
}  // namespace cubao
