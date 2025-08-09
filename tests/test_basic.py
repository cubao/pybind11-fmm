from __future__ import annotations

import pybind11_fmm
from pybind11_fmm import add


def test_add():
    assert add(1, 2) == 3


def test_dummy():
    pybind11_fmm.test_eigen()
