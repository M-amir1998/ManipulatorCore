"""Tests of the optional pybind11 module (skipped unless it was built with -DMC_BUILD_PYTHON=ON and the
build directory is on PYTHONPATH)."""

from __future__ import annotations

import numpy as np
import pytest

mc = pytest.importorskip("manipulatorcore")


def test_forward_kinematics_matches_planar_closed_form():
    robot = mc.make_planar_2r()
    q = np.array([0.3, -0.7])
    T = mc.end_effector_pose(robot, q)
    expected = np.array([np.cos(0.3) + 0.8 * np.cos(-0.4), np.sin(0.3) + 0.8 * np.sin(-0.4), 0.0])
    np.testing.assert_allclose(T[:3, 3], expected, atol=1e-12)


def test_ik_roundtrip_on_seven_dof_arm():
    robot = mc.make_redundant_7r()
    q_true = mc.redundant_7r_home() + 0.1
    target = mc.end_effector_pose(robot, q_true)
    result = mc.solve_ik(robot, target, mc.redundant_7r_home())
    assert result.converged
    np.testing.assert_allclose(mc.end_effector_pose(robot, result.q), target, atol=1e-3)


def test_dynamics_consistency():
    robot = mc.make_industrial_6r()
    q = mc.industrial_6r_home()
    qd = np.full(6, 0.1)
    tau = np.linspace(-5.0, 5.0, 6)
    qdd = mc.forward_dynamics(robot, q, qd, tau)
    np.testing.assert_allclose(mc.inverse_dynamics(robot, q, qd, qdd), tau, atol=1e-9)
    M = mc.mass_matrix(robot, q)
    np.testing.assert_allclose(M, M.T, atol=1e-12)


def test_jacobian_shape_and_urdf():
    robot = mc.make_redundant_7r()
    assert mc.geometric_jacobian(robot, mc.redundant_7r_home()).shape == (6, 7)
    assert "<robot" in mc.to_urdf(robot)
