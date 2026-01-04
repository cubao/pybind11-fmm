#include "types.hpp"

#include <limits>

#include "utils.hpp"

namespace cubao {
Polyline::Polyline(const Eigen::Ref<const RowVectors> &coords, bool is_wgs84)
    : coords_(coords),
      N_(coords.rows()),
      is_wgs84_(is_wgs84),
      k_(is_wgs84 ? cheap_ruler_k(coords(0, 1)) : Eigen::Vector2d::Ones()) {}
Polyline::Polyline(const Eigen::Ref<const RowVectors> &coords, const Eigen::Vector2d &k)
    : coords_(coords), N_(coords.rows()), is_wgs84_(true), k_(k) {}

const Eigen::VectorXd &Polyline::offsets() const { return cache().offsets_; }
double Polyline::offset(int seg_idx, double t) const {
    seg_idx = CLIP(0, seg_idx, N_ - 2);
    auto &offs = offsets();
    return offs[seg_idx] * (1.0 - t) + offs[seg_idx + 1] * t;
}
double Polyline::length() const { return offsets()[N_ - 1]; }

int Polyline::segment_index(double offset) const {
    const double *ptr = offsets().data();
    int i = std::upper_bound(ptr, ptr + N_, offset) - ptr;
    return CLIP(0, i - 1, N_ - 2);
}
std::pair<int, double> Polyline::segment_index_t(double offset) const {
    auto &offs = offsets();
    const double *ptr = offs.data();
    int i = std::upper_bound(ptr, ptr + N_, offset) - ptr;
    i = CLIP(0, i - 1, N_ - 2);
    double t = (offset - offs[i]) / (offs[i + 1] - offs[i]);
    return {i, t};
}

Eigen::Vector2d Polyline::dir(int pt_idx) const { return cache().dirs_.row(CLIP(0, pt_idx, N_ - 1)); }

inline Eigen::Vector2d __smooth_dir(const RowVectors &dirs, int i, double t) {
    if (i == 0) {
        return dirs.row(0);
    } else if (t == 0) {
        Eigen::Vector2d dir = dirs.row(i - 1) + dirs.row(i);
        double len = dir.norm();
        if (len == 0.0) {
            return dirs.row(i);
        }
        return dir / len;
    } else {
        return dirs.row(i);
    }
}

Eigen::Vector2d Polyline::dir(double offset, bool smooth_joint) const {
    if (!smooth_joint) {
        return dir(segment_index(offset));
    }
    auto [i, t] = segment_index_t(offset);
    return __smooth_dir(cache().dirs_, i, t);
}

inline Eigen::Vector2d __interpolate(const Eigen::Vector2d &a, const Eigen::Vector2d &b, double t) {
    return a + (b - a) * t;
}

Eigen::Vector2d Polyline::along(double offset, bool extrapolate) const {
    auto [i, t] = segment_index_t(offset);
    if (!extrapolate) {
        i = CLIP(0, i, N_ - 2);
        t = CLIP(0.0, t, 1.0);
    }
    return __interpolate(coords_.row(i), coords_.row(i + 1), t);
}
std::pair<Eigen::Vector2d, Eigen::Vector2d> Polyline::arrow(double offset, bool extrapolate, bool smooth_joint) const {
    auto [i, t] = segment_index_t(offset);
    if (!extrapolate) {
        i = CLIP(0, i, N_ - 2);
        t = CLIP(0.0, t, 1.0);
    }
    Eigen::Vector2d pos = __interpolate(coords_.row(i), coords_.row(i + 1), t);
    auto &dirs = cache().dirs_;
    Eigen::Vector2d dir = smooth_joint ? (Eigen::Vector2d)dirs.row(i) : __smooth_dir(dirs, i, t);
    return {pos, dir};
}

std::tuple<Eigen::Vector2d, double, int, double> Polyline::nearest(const Eigen::Vector2d &pos, int seg_min,
                                                                   int seg_max) const {
    Eigen::Vector2d PP(0.0, 0.0);
    double dd = std::numeric_limits<double>::max();
    int ss = -1;
    double tt = -1.0;
    if (seg_min < 0) {
        seg_min = 0;
    }
    if (seg_max < 0) {
        seg_max = N_ - 2;
    }
    seg_min = CLIP(0, seg_min, N_ - 2);
    seg_max = CLIP(0, seg_max, N_ - 2);
    if (seg_min > seg_max) {
        return std::make_tuple(PP, dd, ss, tt);
    }
    Eigen::Vector2d xy = pos;
    if (is_wgs84_) {
        xy -= coords_.row(0);
        xy.array() *= k_.array();
    }
    auto &segments = cache().segments_;
    for (int s = seg_min; s <= seg_max; ++s) {
        auto [P, d, t] = segments[s].nearest(xy);
        if (d < dd) {
            PP = P;
            dd = d;
            ss = s;
            tt = t;
        }
    }
    if (is_wgs84_) {
        PP.array() /= k_.array();
        PP += coords_.row(0);
    }
    return {PP, dd, ss, tt};
}

std::tuple<Eigen::Vector2d, double, int, double> Polyline::nearest(const Eigen::Vector2d &pos,  //
                                                                   const Eigen::Vector2d &dir,  //
                                                                   double max_angle_offset,     //
                                                                   int seg_min,                 //
                                                                   int seg_max) const {
    Eigen::Vector2d PP(0.0, 0.0);
    double dd = std::numeric_limits<double>::max();
    int ss = -1;
    double tt = -1.0;
    if (seg_min < 0) {
        seg_min = 0;
    }
    if (seg_max < 0) {
        seg_max = N_ - 2;
    }
    seg_min = CLIP(0, seg_min, N_ - 2);
    seg_max = CLIP(0, seg_max, N_ - 2);
    if (seg_min > seg_max) {
        return std::make_tuple(PP, dd, ss, tt);
    }
    Eigen::Vector2d xy = pos;
    if (is_wgs84_) {
        xy -= coords_.row(0);
        xy.array() *= k_.array();
    }
    double min_dot = std::cos(max_angle_offset * M_PI / 180.0);
    auto &segments = cache().segments_;
    for (int s = seg_min; s <= seg_max; ++s) {
        if (dir.dot(segments[s].dir) < min_dot) {
            continue;
        }
        auto [P, d, t] = segments[s].nearest(xy);
        if (d < dd) {
            PP = P;
            dd = d;
            ss = s;
            tt = t;
        }
    }
    if (is_wgs84_) {
        PP.array() /= k_.array();
        PP += coords_.row(0);
    }
    return {PP, dd, ss, tt};
}

void __norms_dirs(const RowVectors &polyline, int N, Eigen::VectorXd &norms, RowVectors &dirs) {
    constexpr double eps = std::numeric_limits<double>::min();
    dirs = polyline.bottomRows(N - 1) - polyline.topRows(N - 1);  // deltas
    norms = dirs.rowwise().norm();

    // dirs = delta / len(delta)
    for (int i = 0; i < N - 1; ++i) {
        if (norms[i]) {
            dirs.row(i) /= norms[i];
            continue;
        }
        // try find left, right effective-offset nodes
        int l = i, r = i + 1;
        while (r < N - 1 && !norms[r]) {
            ++r;
        }
        if (r != i + 1 || r == N) {
            while (l >= 0 && !norms[l]) {
                --l;
            }
        }
        l = std::max(0, l);
        r = std::min(N - 1, r);
        if (!norms.segment(l, r - l + 1).sum()) {
            dirs.row(i).setZero();
            continue;
        }
        Eigen::Vector2d delta = polyline.row(std::min(r + 1, N - 1)) - polyline.row(l);
        dirs.row(i) = delta / delta.norm();
    }
}

const Polyline::Cache &Polyline::cache() const {
    if (cache_) {
        return *cache_;
    }
    Cache cache;
    Eigen::VectorXd norms;
    auto &dirs = cache.dirs_;
    auto &segs = cache.segments_;
    segs.reserve(N_ - 1);
    if (is_wgs84_) {
        auto xys = wgs84_to_xy(coords_, coords_.row(0), k_);
        __norms_dirs(xys, N_, norms, dirs);
        for (int i = 1; i < N_; ++i) {
            segs.emplace_back(xys.row(i - 1), xys.row(i),  //
                              dirs.row(i - 1),             //
                              norms[i - 1]);
        }
    } else {
        __norms_dirs(coords_, N_, norms, dirs);
        for (int i = 1; i < N_; ++i) {
            segs.emplace_back(coords_.row(i - 1), coords_.row(i),  //
                              dirs.row(i - 1),                     //
                              norms[i - 1]);
        }
    }
    auto &offs = cache.offsets_;
    offs = Eigen::VectorXd(N_);
    offs[0] = 0.0;
    for (int i = 1; i < N_; ++i) {
        offs[i] = offs[i - 1] + norms[i - 1];
    }
    cache_ = std::move(cache);
    return *cache_;
}

const Eigen::Vector4d &Polyline::bbox() const {
    if (bbox_) {
        return *bbox_;
    }

    // Calculate bounding box (minX, minY, maxX, maxY)
    double minX = std::numeric_limits<double>::infinity();
    double minY = std::numeric_limits<double>::infinity();
    double maxX = -std::numeric_limits<double>::infinity();
    double maxY = -std::numeric_limits<double>::infinity();

    for (int i = 0; i < N_; ++i) {
        double x = coords_(i, 0);
        double y = coords_(i, 1);
        minX = std::min(minX, x);
        minY = std::min(minY, y);
        maxX = std::max(maxX, x);
        maxY = std::max(maxY, y);
    }

    bbox_ = Eigen::Vector4d(minX, minY, maxX, maxY);
    return *bbox_;
}

}  // namespace cubao
