"""pybind11_fmm - Fast Map Matching with C++

This package provides a high-performance implementation of the Fast Map Matching (FMM)
algorithm for matching GPS trajectories to road networks.
"""

from __future__ import annotations

import numpy as np

from ._core import Network, ProjectedPoint, fmm

__all__ = [
    "Network",
    "ProjectedPoint",
    "FastMapMatchConfig",
    "FastMapMatch",
    "fmm",
]


class FastMapMatchConfig:
    """Configuration for Fast Map Matching algorithm.

    This class is compatible with topo-graph's FastMapMatchConfig interface.

    Attributes:
        k: Maximum number of candidate points to consider for each GPS point.
        radius: Search radius in meters for finding candidate road segments.
        gps_error: Standard deviation of GPS measurement error in meters.
        reverse_tolerance: Tolerance for matching paths that go in reverse direction.
    """

    def __init__(
        self,
        k: int = 50,
        radius: float = 160.0,
        gps_error: float = 40.0,
        reverse_tolerance: float = 0.0,
    ):
        self._config = fmm.Config()
        self._config.k = k
        self._config.radius = radius
        self._config.gps_error = gps_error
        self._config.reverse_tolerance = reverse_tolerance

    @property
    def k(self) -> int:
        return self._config.k

    @k.setter
    def k(self, value: int):
        self._config.k = value

    @property
    def radius(self) -> float:
        return self._config.radius

    @radius.setter
    def radius(self, value: float):
        self._config.radius = value

    @property
    def gps_error(self) -> float:
        return self._config.gps_error

    @gps_error.setter
    def gps_error(self, value: float):
        self._config.gps_error = value

    @property
    def reverse_tolerance(self) -> float:
        return self._config.reverse_tolerance

    @reverse_tolerance.setter
    def reverse_tolerance(self, value: float):
        self._config.reverse_tolerance = value


class FastMapMatch:
    """Fast Map Matching algorithm.

    This class provides the main interface for matching GPS trajectories to road networks.
    It's compatible with topo-graph's FastMapMatch interface.

    Args:
        network: The road network to match against.
        config: Configuration parameters (optional).

    Examples:
        >>> import numpy as np
        >>> from pybind11_fmm import Network, FastMapMatch, FastMapMatchConfig
        >>>
        >>> # Build network
        >>> network = Network()
        >>> network.add_edge(1, np.array([[0.0, 0.0], [1.0, 1.0]]), is_wgs84=False)
        >>>
        >>> # Match trajectory
        >>> fmm = FastMapMatch(network)
        >>> trajectory = np.array([[0.1, 0.1], [0.9, 0.9]])
        >>> result = fmm.match_traj(trajectory)
        >>> print(f"Matched path: {result.optimal_path}")
    """

    def __init__(
        self,
        network: Network,
        config: FastMapMatchConfig | None = None,
    ):
        self.network = network
        self.config = config if config is not None else FastMapMatchConfig()

    def match_traj(self, trajectory: np.ndarray) -> fmm.MatchResult:
        """Match GPS trajectory to road network.

        Args:
            trajectory: Nx2 numpy array of GPS points (lon/lat or x/y).

        Returns:
            MatchResult containing the matched path and detailed information.

        Raises:
            ValueError: If trajectory is not Nx2 array.
        """
        # Validate input
        trajectory = np.asarray(trajectory, dtype=np.float64)
        if trajectory.ndim != 2 or trajectory.shape[1] != 2:
            msg = f"Trajectory must be Nx2 array, got shape {trajectory.shape}"
            raise ValueError(msg)

        if len(trajectory) == 0:
            # Return empty result for empty trajectory
            result = fmm.MatchResult()
            result.success = False
            result.score = float("-inf")
            return result

        # Candidate search
        candidates = fmm.search_candidates(
            self.network, trajectory, self.config._config
        )

        # HMM matching
        return fmm.match_trajectory(
            self.network, trajectory, candidates, self.config._config
        )
