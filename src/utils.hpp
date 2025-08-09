#pragma once

#include "types.hpp"

namespace cubao {

inline double ROUND(double v, double s) {
    return std::floor(v * s + 0.5) / s;  // not same std::round(v * s) / s;
}

inline double ROUND(double v, std::optional<double> s) { return s ? ROUND(v, *s) : v; }

template <typename T>
T CLIP(T low, T v, T high) {
    return v < low ? low : (v > high ? high : v);
}

// https://github.com/cubao/headers/blob/main/include/cubao/crs_transform.hpp
inline Eigen::Vector2d cheap_ruler_k(double latitude) {
    // based on https://github.com/mapbox/cheap-ruler-cpp
    static constexpr double RE = 6378.137;
    static constexpr double FE = 1.0 / 298.257223563;
    static constexpr double E2 = FE * (2.0 - FE);
    static constexpr double RAD = M_PI / 180.0;
    static constexpr double MUL = RAD * RE * 1000.0;
    double coslat = std::cos(latitude * RAD);
    double w2 = 1.0 / (1.0 - E2 * (1.0 - coslat * coslat));
    double w = std::sqrt(w2);
    return Eigen::Vector2d(MUL * w * coslat, MUL * w * w2 * (1.0 - E2));
}

inline RowVectors wgs84_to_xy(const Eigen::Ref<const RowVectors> &wgs84,   //
                              std::optional<Eigen::Vector2d> anchor = {},  //
                              std::optional<Eigen::Vector2d> k = {}) {
    if (!anchor) {
        anchor = wgs84.row(0);
    }
    if (!k) {
        k = cheap_ruler_k((*anchor)[1]);
    }
    RowVectors xys = wgs84;
    for (int i = 0; i < 2; ++i) {
        xys.col(i).array() -= (*anchor)[i];
        xys.col(i).array() *= (*k)[i];
    }
    return xys;
}

inline RowVectors xy_to_wgs84(const Eigen::Ref<const RowVectors> &xys,  //
                              const Eigen::Vector2d &anchor,            //
                              std::optional<Eigen::Vector2d> k = {}) {
    if (!k) {
        k = cheap_ruler_k(anchor[1]);
    }
    RowVectors wgs84 = xys;
    for (int i = 0; i < 2; ++i) {
        wgs84.col(i).array() /= (*k)[i];
        wgs84.col(i).array() += anchor[i];
    }
    return wgs84;
}

inline Eigen::Vector2d heading_direction(double heading) {
    double yaw = (90.0 - heading) / 180.0 * M_PI;
    return Eigen::Vector2d(std::cos(yaw), std::sin(yaw));
}

inline bool starts_with(const std::vector<int64_t> &nodes, const std::vector<int64_t> &prefix) {
    return !prefix.empty() &&                //
           prefix.size() <= nodes.size() &&  //
           std::equal(prefix.begin(), prefix.end(), nodes.begin());
}

inline bool ends_with(const std::vector<int64_t> &nodes, const std::vector<int64_t> &suffix) {
    return !suffix.empty() &&                //
           suffix.size() <= nodes.size() &&  //
           std::equal(suffix.begin(), suffix.end(), &nodes[nodes.size() - suffix.size()]);
}

RowVectors douglas_simplify(const Eigen::Ref<const RowVectors> &coords, double epsilon, bool is_wgs84 = false);

std::string encode_polyline(const Eigen::Ref<const RowVectors> &coords);
RowVectors decode_polyline(const std::string &encoded);

}  // namespace cubao
