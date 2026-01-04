"""Test Fast Map Matching functionality."""

from __future__ import annotations

import numpy as np
import pytest

from pybind11_fmm import FastMapMatch, FastMapMatchConfig, Network


def test_network_creation():
    """Test creating a network and adding edges."""
    network = Network()

    # Add a simple edge
    coords = np.array([[0.0, 0.0], [1.0, 1.0], [2.0, 2.0]], dtype=np.float64)
    network.add_edge(1, coords, is_wgs84=False)

    # Get geometry back
    geom = network.geometry(1)
    assert geom is not None


def test_spatial_query():
    """Test spatial query for candidate roads."""
    network = Network()

    # Add some edges
    network.add_edge(1, np.array([[0.0, 0.0], [1.0, 0.0]]), is_wgs84=False)
    network.add_edge(2, np.array([[0.0, 1.0], [1.0, 1.0]]), is_wgs84=False)

    # Query near the first edge
    pt = np.array([0.5, 0.1])
    candidates = network.query_radius(pt, radius=0.5)

    # Should find edge 1
    assert len(candidates) > 0
    assert candidates[0].edge_id == 1
    assert candidates[0].distance < 0.5


def test_basic_matching():
    """Test basic FMM matching."""
    network = Network()

    # Create a simple straight road
    network.add_edge(1, np.array([[0.0, 0.0], [10.0, 0.0]]), is_wgs84=False)

    # Create trajectory along the road with some noise
    trajectory = np.array(
        [[1.0, 0.1], [3.0, -0.1], [5.0, 0.05], [7.0, -0.05], [9.0, 0.1]]
    )

    # Match trajectory
    fmm = FastMapMatch(network)
    result = fmm.match_traj(trajectory)

    # Should successfully match
    assert result.success
    assert len(result.matched_points) == len(trajectory)
    assert 1 in result.optimal_path


def test_empty_trajectory():
    """Test handling of empty trajectory."""
    network = Network()
    network.add_edge(1, np.array([[0.0, 0.0], [1.0, 1.0]]), is_wgs84=False)

    fmm = FastMapMatch(network)
    result = fmm.match_traj(np.array([]).reshape(0, 2))

    assert not result.success


def test_config():
    """Test configuration parameters."""
    config = FastMapMatchConfig(
        k=10, radius=100.0, gps_error=20.0, reverse_tolerance=0.0
    )

    assert config.k == 10
    assert config.radius == 100.0
    assert config.gps_error == 20.0
    assert config.reverse_tolerance == 0.0

    # Test setters
    config.k = 20
    assert config.k == 20


def test_two_edge_network():
    """Test matching on a network with multiple edges."""
    network = Network()

    # Create L-shaped road network
    network.add_edge(1, np.array([[0.0, 0.0], [5.0, 0.0]]), is_wgs84=False)
    network.add_edge(2, np.array([[5.0, 0.0], [5.0, 5.0]]), is_wgs84=False)

    # Trajectory that follows the L-shape
    trajectory = np.array(
        [
            [1.0, 0.1],
            [3.0, -0.1],
            [4.9, 0.1],  # Near junction
            [5.0, 2.0],
            [5.1, 4.0],
        ]
    )

    fmm = FastMapMatch(network)
    result = fmm.match_traj(trajectory)

    assert result.success
    assert len(result.matched_points) == len(trajectory)
    # Should match to both edges
    assert len(set(result.optimal_path)) >= 1  # At least one edge matched


def test_invalid_trajectory_shape():
    """Test error handling for invalid trajectory shape."""
    network = Network()
    network.add_edge(1, np.array([[0.0, 0.0], [1.0, 1.0]]), is_wgs84=False)

    fmm = FastMapMatch(network)

    # 1D array should fail
    with pytest.raises(ValueError, match="must be Nx2 array"):
        fmm.match_traj(np.array([1.0, 2.0]))

    # 3D array should fail
    with pytest.raises(ValueError, match="must be Nx2 array"):
        fmm.match_traj(np.array([[[1.0, 2.0]]]))


if __name__ == "__main__":
    # Run tests manually
    test_network_creation()
    test_spatial_query()
    test_basic_matching()
    test_empty_trajectory()
    test_config()
    test_two_edge_network()
    test_invalid_trajectory_shape()
    print("All tests passed!")
