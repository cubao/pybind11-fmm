"""

pybind11_fmm - Fast Map Matching with C++
------------------------------------------

Fast Map Matching (FMM) algorithm for matching GPS trajectories to road networks.

"""

from __future__ import annotations

import typing

import numpy
import numpy.typing

from . import fmm

__all__: list[str] = ["Network", "Polyline", "ProjectedPoint", "fmm"]

class Network:
    def __init__(self) -> None: ...
    def add_edge(
        self,
        edge_id: typing.SupportsInt,
        coords: typing.Annotated[
            numpy.typing.NDArray[numpy.float64], "[m, 2]", "flags.c_contiguous"
        ],
        is_wgs84: bool = True,
    ) -> None:
        """
        Add an edge to the network
        """
    def geometry(self, edge_id: typing.SupportsInt) -> Polyline:
        """
        Get the geometry of an edge
        """
    def query_radius(
        self,
        pt: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[2, 1]"],
        radius: typing.SupportsFloat,
    ) -> list[ProjectedPoint]:
        """
        Query edges within radius of a point
        """

class Polyline:
    def bbox(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[4, 1]"]: ...
    def length(self) -> float: ...
    @property
    def coords_(
        self,
    ) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 2]"]: ...

class ProjectedPoint:
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

__version__: str = "0.0.1"
