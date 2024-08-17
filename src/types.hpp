#pragma once

#include <Eigen/Core>
#include <optional>
#include <string>

#include "ankerl/unordered_dense.h"

#define _USE_MATH_DEFINES
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace cubao {
// coords type, 2d
using RowVectors = Eigen::Matrix<double, Eigen::Dynamic, 2, Eigen::RowMajor>;

// kv type
template <typename Key, typename Value, typename Hash = ankerl::unordered_dense::hash<Key>,
          typename Equal = std::equal_to<Key>>
using unordered_map = ankerl::unordered_dense::map<Key, Value, Hash, Equal>;
template <typename Value, typename Hash = ankerl::unordered_dense::hash<Value>, typename Equal = std::equal_to<Value>>
using unordered_set = ankerl::unordered_dense::set<Value, Hash, Equal>;

// string indexer

struct Indexer {
    Indexer() = default;
    Indexer(const std::map<std::string, int64_t> &index) { this->index(index); }
    bool contains(const std::string &id) const { return str2int_.find(id) != str2int_.end(); }
    bool contains(int64_t id) const { return int2str_.find(id) != int2str_.end(); }
    std::optional<std::string> get_id(int64_t id) const {
        auto itr = int2str_.find(id);
        if (itr == int2str_.end()) {
            return {};
        }
        return itr->second;
    }
    std::optional<int64_t> get_id(const std::string &id) const {
        auto itr = str2int_.find(id);
        if (itr == str2int_.end()) {
            return {};
        }
        return itr->second;
    }

    // get str id
    std::string id(int64_t id) const { return int2str_.at(id); }
    // get str id (with auto setup)
    std::string id(int64_t id) {
        auto itr = int2str_.find(id);
        if (itr != int2str_.end()) {
            return itr->second;
        }
        int round = 0;
        auto id_str = std::to_string(id);
        auto str_id = id_str;
        while (str2int_.count(str_id)) {
            ++round;
            str_id = id_str + "/" + std::to_string(round);
        }
        index(str_id, id);
        return str_id;
    }
    // get int id
    int64_t id(const std::string &id) const { return str2int_.at(id); }
    // get int id (with auto setup)
    int64_t id(const std::string &id) {
        auto itr = str2int_.find(id);
        if (itr != str2int_.end()) {
            return itr->second;
        }
        try {
            // '44324' -> 44324
            // 'w44324' -> 44324
            int64_t ii = id[0] == 'w' ? std::stoll(id.substr(1)) : std::stoll(id);
            if (index(id, ii)) {
                return ii;
            }
        } catch (...) {
        }
        while (!index(id, id_cursor_)) {
            ++id_cursor_;
        }
        return id_cursor_++;
    }
    // setup str/int id, returns true (setup) or false (skip)
    bool index(const std::string &str_id, int64_t int_id) {
        if (str2int_.count(str_id) || int2str_.count(int_id)) {
            return false;
        }
        str2int_.emplace(str_id, int_id);
        int2str_.emplace(int_id, str_id);
        return true;
    }
    bool index(const std::map<std::string, int64_t> &index) {
        bool succ = true;
        for (auto &kv : index) {
            succ &= this->index(kv.first, kv.second);
        }
        return succ;
    }
    std::map<std::string, int64_t> index() const { return {str2int_.begin(), str2int_.end()}; }

  private:
    unordered_map<std::string, int64_t> str2int_;
    unordered_map<int64_t, std::string> int2str_;
    int64_t id_cursor_{1000000};
};

// 2d line segment
struct LineSegment {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    const Eigen::Vector2d A, B, dir;
    const double len;
    LineSegment(const Eigen::Vector2d &a, const Eigen::Vector2d &b)
        : A(a), B(b), dir((b - a).normalized()), len((b - a).norm()) {}
    LineSegment(const Eigen::Vector2d &a, const Eigen::Vector2d &b, const Eigen::Vector2d &dir, double length)
        : A(a), B(b), dir(dir), len(length) {}

    double dist(const Eigen::Vector2d &P) const { return std::sqrt(dist2(P)); }
    double dist2(const Eigen::Vector2d &P) const {
        double dot = (P - A).dot(dir);
        if (dot <= 0) {
            return (P - A).squaredNorm();
        } else if (dot >= len) {
            return (P - B).squaredNorm();
        }
        return (A + dot * dir - P).squaredNorm();
    }

    std::tuple<Eigen::Vector3d, double, double> nearest(const Eigen::Vector2d &P) const {
        double dot = (P - A).dot(dir);
        if (dot <= 0) {
            return std::make_tuple(A, (P - A).norm(), 0.0);
        } else if (dot >= len) {
            return std::make_tuple(B, (P - B).norm(), 1.0);
        }
        Eigen::Vector2d PP = A + dot * dir;
        return std::make_tuple(PP, (PP - P).norm(), dot / len);
    }
};

// 2d polyline, supports XY (cartesian) and WGS84
struct Polyline {
    // TODO
};

}  // namespace cubao
