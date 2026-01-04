"""
Fast Map Matching
"""

from __future__ import annotations

import collections.abc
import typing

import numpy
import numpy.typing

import pybind11_fmm._core

__all__: list[str] = [
    "Candidate",
    "Config",
    "MatchResult",
    "MatchedCandidate",
    "match_trajectory",
    "search_candidates",
]

class Candidate:
    @property
    def distance(self) -> float: ...
    @property
    def edge_id(self) -> int: ...
    @property
    def offset(self) -> float: ...
    @property
    def point(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[2, 1]"]: ...

class Config:
    def __init__(self) -> None: ...
    @property
    def gps_error(self) -> float: ...
    @gps_error.setter
    def gps_error(self, arg0: typing.SupportsFloat) -> None: ...
    @property
    def k(self) -> int: ...
    @k.setter
    def k(self, arg0: typing.SupportsInt) -> None: ...
    @property
    def radius(self) -> float: ...
    @radius.setter
    def radius(self, arg0: typing.SupportsFloat) -> None: ...
    @property
    def reverse_tolerance(self) -> float: ...
    @reverse_tolerance.setter
    def reverse_tolerance(self, arg0: typing.SupportsFloat) -> None: ...

class MatchResult:
    success: bool
    def __init__(self) -> None: ...
    @property
    def matched_points(self) -> list[MatchedCandidate]: ...
    @matched_points.setter
    def matched_points(
        self, arg0: collections.abc.Sequence[MatchedCandidate]
    ) -> None: ...
    @property
    def optimal_path(self) -> list[int]: ...
    @optimal_path.setter
    def optimal_path(
        self, arg0: collections.abc.Sequence[typing.SupportsInt]
    ) -> None: ...
    @property
    def score(self) -> float: ...
    @score.setter
    def score(self, arg0: typing.SupportsFloat) -> None: ...

class MatchedCandidate:
    @property
    def edge_id(self) -> int: ...
    @property
    def offset(self) -> float: ...
    @property
    def probability(self) -> float: ...

def match_trajectory(
    network: pybind11_fmm._core.Network,
    trajectory: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, 2]"],
    candidates: collections.abc.Sequence[collections.abc.Sequence[Candidate]],
    config: Config,
) -> MatchResult:
    """
    Match GPS trajectory to road network using HMM
    """

def search_candidates(
    network: pybind11_fmm._core.Network,
    trajectory: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, 2]"],
    config: Config,
) -> list[list[Candidate]]:
    """
    Search for candidate road segments for each GPS point
    """
