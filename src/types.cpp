#include "types.hpp"
#include "utils.hpp"
#include <limits>

namespace cubao {
Polyline::Polyline(const Eigen::Ref<const RowVectors> &coords, bool is_wgs84)
    : coords_(coords),
      N_(coords.rows()),
      is_wgs84_(is_wgs84),
      k_(is_wgs84 ? cheap_ruler_k(coords(0, 1)) : Eigen::Vector2d::Ones()) {}
Polyline::Polyline(const Eigen::Ref<const RowVectors> &coords, const Eigen::Vector2d &k)
    : coords_(coords), N_(coords.rows()), is_wgs84_(true), k_(k) {}


    const Eigen::VectorXd &Polyline::offsets() const {
        return cache().offsets_;
    }
    double Polyline::offset(int seg_idx, double t) const {
        seg_idx = CLIP(0, seg_idx, N_ - 2);
        auto &offs = offsets();
        return offs[seg_idx] * (1.0 - t) + offs[seg_idx + 1] * t;
    }
    double Polyline::length() const {
        return offsets()[N_ - 1];
    }


inline Eigen::VectorXd __offsets(const RowVectors &polyline, double eps = std::numeric_limits<double>::min())
{
    const int N = polyline.rows();
    Eigen::VectorXd norms =
        (polyline.bottomRows(N - 1) - polyline.topRows(N - 1)).rowwise().norm();
    Eigen::VectorXd offs(N);
    offs[0] = 0.0;
    for (int i = 1; i < N; ++i) {
        offs[i] = offs[i - 1] + std::max(eps, norms[i - 1]);
    }
    return offs;
}

inline RowVectors __dirs(const RowVectors &polyline)
{
    //                             o
    //        dir0   dir1   dir2  / dir3
    //      o------o------o------o
    const int N = polyline.rows();
    RowVectors dirs = polyline.bottomRows(N - 1) - polyline.topRows(N - 1);
    Eigen::VectorXd norms2 = (polyline.bottomRows(N - 1) -
                              polyline.topRows(N - 1))
                                 .rowwise()
                                 .norm();
    for (int i = 0; i < N - 1; ++i) {
        if (norms2[i]) {
            dirs.row(i) /= dirs.row(i).norm();
            continue;
        }
        // try find left, right effective-offset nodes
        int l = i, r = i + 1;
        while (r < N - 1 && !norms2[r]) {
            ++r;
        }
        if (r != i + 1 || r == N) {
            while (l >= 0 && !norms2[l]) {
                --l;
            }
        }
        l = std::max(0, l);
        r = std::min(N - 1, r);
        if (!norms2.segment(l, r - l + 1).sum()) {
            throw std::invalid_argument("polyline is collapsed under plane-xy");
        }
        Eigen::Vector3d delta =
            polyline.row(std::min(r + 1, N - 1)) - polyline.row(l);
        dirs.row(i) = delta / delta.norm();
    }
    return dirs;
}



}  // namespace cubao