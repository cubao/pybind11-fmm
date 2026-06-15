#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>

#include <Eigen/Core>

// #define DBG_MACRO_DISABLE
#define DBG_MACRO_NO_WARNING
#include <iostream>
#include <limits>
#include <map>
#include <unordered_map>
#include <vector>

#include "dbg.h"
#include "fmm/fmm.hpp"
#include "network.hpp"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

int add(int i, int j) { return i + j; }

namespace nb = nanobind;
using namespace nb::literals;

NB_MODULE(_core, m) {
    using namespace cubao;

    m.doc() = R"pbdoc(
        pybind11_fmm - Fast Map Matching with C++
        ------------------------------------------

        Fast Map Matching (FMM) algorithm for matching GPS trajectories to road networks.
    )pbdoc";

    // Polyline
    nb::class_<Polyline>(m, "Polyline")
        .def_ro("coords_", &Polyline::coords_)
        .def("length", &Polyline::length)
        .def("bbox", &Polyline::bbox);

    // ProjectedPoint
    nb::class_<ProjectedPoint>(m, "ProjectedPoint")
        .def_ro("edge_id", &ProjectedPoint::edge_id)
        .def_ro("offset", &ProjectedPoint::offset)
        .def_ro("distance", &ProjectedPoint::distance)
        .def_ro("point", &ProjectedPoint::point);

    // Network
    nb::class_<Network>(m, "Network")
        .def(nb::init<>())
        .def("add_edge", &Network::add_edge, "edge_id"_a, "coords"_a, "is_wgs84"_a = true, "Add an edge to the network")
        .def("geometry", &Network::geometry, "edge_id"_a, nb::rv_policy::reference_internal,
             "Get the geometry of an edge")
        .def("query_radius", &Network::query_radius, "pt"_a, "radius"_a, "Query edges within radius of a point");

    // FMM submodule
    auto fmm = m.def_submodule("fmm", "Fast Map Matching");

    // FMM Config
    nb::class_<fmm::Config>(fmm, "Config")
        .def(nb::init<>())
        .def_rw("k", &fmm::Config::k)
        .def_rw("radius", &fmm::Config::radius)
        .def_rw("gps_error", &fmm::Config::gps_error)
        .def_rw("reverse_tolerance", &fmm::Config::reverse_tolerance);

    // FMM Candidate
    nb::class_<fmm::Candidate>(fmm, "Candidate")
        .def_ro("edge_id", &fmm::Candidate::edge_id)
        .def_ro("offset", &fmm::Candidate::offset)
        .def_ro("distance", &fmm::Candidate::distance)
        .def_ro("point", &fmm::Candidate::point);

    // FMM MatchedCandidate
    nb::class_<fmm::MatchedCandidate>(fmm, "MatchedCandidate")
        .def_ro("edge_id", &fmm::MatchedCandidate::edge_id)
        .def_ro("offset", &fmm::MatchedCandidate::offset)
        .def_ro("probability", &fmm::MatchedCandidate::probability);

    // FMM MatchResult
    nb::class_<fmm::MatchResult>(fmm, "MatchResult")
        .def(nb::init<>())
        .def_rw("matched_points", &fmm::MatchResult::matched_points)
        .def_rw("optimal_path", &fmm::MatchResult::optimal_path)
        .def_rw("score", &fmm::MatchResult::score)
        .def_rw("success", &fmm::MatchResult::success);

    // FMM functions
    fmm.def("search_candidates", &fmm::search_candidates, "network"_a, "trajectory"_a, "config"_a,
            "Search for candidate road segments for each GPS point");

    fmm.def("match_trajectory", &fmm::match_trajectory, "network"_a, "trajectory"_a, "candidates"_a, "config"_a,
            "Match GPS trajectory to road network using HMM");

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
