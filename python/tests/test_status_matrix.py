"""The README status matrix must match configs/algorithms.yaml, and every referenced file must exist."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]

pytest.importorskip("yaml")


def test_status_matrix_is_up_to_date():
    result = subprocess.run(
        [sys.executable, str(REPO / "scripts" / "docs" / "generate_status_matrix.py"), "--check"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
