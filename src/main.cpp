#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

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

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(_core, m) {
    using namespace cubao;

    m.doc() = R"pbdoc(
        pybind11_fmm - Fast Map Matching with C++
        ------------------------------------------

        Fast Map Matching (FMM) algorithm for matching GPS trajectories to road networks.
    )pbdoc";

    // Polyline
    py::class_<Polyline>(m, "Polyline")
        .def_readonly("coords_", &Polyline::coords_)
        .def("length", &Polyline::length)
        .def("bbox", &Polyline::bbox);

    // ProjectedPoint
    py::class_<ProjectedPoint>(m, "ProjectedPoint")
        .def_readonly("edge_id", &ProjectedPoint::edge_id)
        .def_readonly("offset", &ProjectedPoint::offset)
        .def_readonly("distance", &ProjectedPoint::distance)
        .def_readonly("point", &ProjectedPoint::point);

    // Network
    py::class_<Network>(m, "Network")
        .def(py::init<>())
        .def("add_edge", &Network::add_edge, py::arg("edge_id"), py::arg("coords"), py::arg("is_wgs84") = true,
             "Add an edge to the network")
        .def("geometry", &Network::geometry, py::arg("edge_id"), py::return_value_policy::reference_internal,
             "Get the geometry of an edge")
        .def("query_radius", &Network::query_radius, py::arg("pt"), py::arg("radius"),
             "Query edges within radius of a point");

    // FMM submodule
    auto fmm = m.def_submodule("fmm", "Fast Map Matching");

    // FMM Config
    py::class_<fmm::Config>(fmm, "Config")
        .def(py::init<>())
        .def_readwrite("k", &fmm::Config::k)
        .def_readwrite("radius", &fmm::Config::radius)
        .def_readwrite("gps_error", &fmm::Config::gps_error)
        .def_readwrite("reverse_tolerance", &fmm::Config::reverse_tolerance);

    // FMM Candidate
    py::class_<fmm::Candidate>(fmm, "Candidate")
        .def_readonly("edge_id", &fmm::Candidate::edge_id)
        .def_readonly("offset", &fmm::Candidate::offset)
        .def_readonly("distance", &fmm::Candidate::distance)
        .def_readonly("point", &fmm::Candidate::point);

    // FMM MatchedCandidate
    py::class_<fmm::MatchedCandidate>(fmm, "MatchedCandidate")
        .def_readonly("edge_id", &fmm::MatchedCandidate::edge_id)
        .def_readonly("offset", &fmm::MatchedCandidate::offset)
        .def_readonly("probability", &fmm::MatchedCandidate::probability);

    // FMM MatchResult
    py::class_<fmm::MatchResult>(fmm, "MatchResult")
        .def(py::init<>())
        .def_readwrite("matched_points", &fmm::MatchResult::matched_points)
        .def_readwrite("optimal_path", &fmm::MatchResult::optimal_path)
        .def_readwrite("score", &fmm::MatchResult::score)
        .def_readwrite("success", &fmm::MatchResult::success);

    // FMM functions
    fmm.def("search_candidates", &fmm::search_candidates, py::arg("network"), py::arg("trajectory"), py::arg("config"),
            "Search for candidate road segments for each GPS point");

    fmm.def("match_trajectory", &fmm::match_trajectory, py::arg("network"), py::arg("trajectory"),
            py::arg("candidates"), py::arg("config"), "Match GPS trajectory to road network using HMM");

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
