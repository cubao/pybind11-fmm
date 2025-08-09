# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Project**: pybind11-fmm - Fast Marching Method (FMM) implementation with Python bindings
**Core Technologies**: C++17, Python 3.7+, pybind11, Eigen3, scikit-build-core

## Quick Commands

### Development
```bash
# Build development version
make build

# Install package locally
make python_install

# Run tests
make pytest

# Build distribution wheels
make python_wheel

# Clean build artifacts
make clean
```

### Testing
```bash
# Run all tests
pytest

# Run with output
pytest -rP tests/

# Run specific test
pytest tests/test_basic.py::test_add
```

### Code Quality
- **C++**: `clang-format` (auto-runs via pre-commit)
- **Python**: `ruff` (auto-runs via pre-commit)
- **Pre-commit**: Runs automatically on commit, or manually with `pre-commit run --all-files`

## Architecture

### Core Components
- **src/main.cpp**: Pybind11 module entry point
- **src/graph.***: Directed graph data structures and algorithms
- **src/network.***: Network data structures with RTree spatial indexing
- **src/types.***: Core mathematical types (RowVectors, LineSegment, Polyline)
- **src/utils.***: Utility functions and helpers

### Key Algorithms
- Shortest path (Dijkstra, A*)
- Zigzag path (bidirectional routing)
- Routing with binding constraints
- UBODT (Upper Bound Origin-Destination Table) construction

### Third-party Dependencies
- **Eigen3** (in `src/3rdparty/Eigen/`): Linear algebra
- **ankerl/unordered_dense**: High-performance hash maps
- **dbg.h**: Debugging utilities

## Build System

### CMake Configuration
- **C++ Standard**: C++17
- **Build Types**: Debug (-O0, -ggdb) and Release (-O3)
- **Dependencies**: Eigen3 (header-only), pybind11
- **Output**: Python extension module (`pybind11_fmm.*.so`)

### Python Packaging
- **Backend**: scikit-build-core
- **Requirements**: Python ≥3.7
- **Distribution**: Source + binary wheels for all platforms

## File Structure
```
src/
├── main.cpp              # Pybind11 module definition
├── graph.cpp/.hpp        # Graph algorithms
├── network.cpp/.hpp      # Network with spatial indexing
├── types.cpp/.hpp        # Core geometric types
├── utils.cpp/.hpp        # Utilities
├── 3rdparty/            # Third-party dependencies
└── pybind11_fmm/        # Python package files
```

## CI/CD
- **GitHub Actions**: `.github/workflows/`
- **Platforms**: Windows, macOS, Ubuntu
- **Python**: 3.7-3.12
- **Publishing**: Automatic PyPI release on GitHub releases
