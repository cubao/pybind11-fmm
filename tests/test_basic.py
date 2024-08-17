from __future__ import annotations

import fmmpy
from fmmpy import add


def test_add():
    assert add(1, 2) == 3


def test_dummy():
    fmmpy.test_eigen()
