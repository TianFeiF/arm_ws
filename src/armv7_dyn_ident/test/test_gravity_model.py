# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
"""Gravity model correctness: regressor linearity + match to energy gradient."""
import numpy as np

from armv7_dyn_ident.gravity_model import GRAVITY


def _numeric_potential(gm, q):
    """Independent numpy FK + potential energy, used to finite-difference G(q)."""
    def rpy(r, p, y):
        cr, sr = np.cos(r), np.sin(r)
        cp, sp = np.cos(p), np.sin(p)
        cy, sy = np.cos(y), np.sin(y)
        rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        return rz @ ry @ rx

    def axrot(axis, a):
        ax = np.asarray(axis, float)
        ax = ax / np.linalg.norm(ax)
        x, yy, z = ax
        c, s = np.cos(a), np.sin(a)
        v = 1 - c
        return np.array([
            [x * x * v + c, x * yy * v - z * s, x * z * v + yy * s],
            [x * yy * v + z * s, yy * yy * v + c, yy * z * v - x * s],
            [x * z * v - yy * s, yy * z * v + x * s, z * z * v + c]])

    T = np.eye(4)
    u = 0.0
    g = np.array([0, 0, -GRAVITY])
    for i, s in enumerate(gm.chain):
        to = np.eye(4)
        to[:3, :3] = rpy(*s.joint.rpy)
        to[:3, 3] = s.joint.xyz
        tr = np.eye(4)
        tr[:3, :3] = axrot(s.joint.axis, q[i])
        T = T @ to @ tr
        pci = T[:3, :3] @ np.asarray(s.link.com) + T[:3, 3]
        u += -g @ (s.link.mass * pci)
    return u


def test_chain_is_seven_revolute(gm):
    assert gm.n == 7
    assert gm.joint_names == [f'joint{i}' for i in range(1, 8)]


def test_regressor_is_linear_in_params(gm):
    rng = np.random.default_rng(0)
    phi = gm.urdf_params()
    for _ in range(10):
        q = rng.uniform(-1.5, 1.5, gm.n)
        assert np.allclose(gm.regressor(q) @ phi, gm.torque(q, phi))


def test_torque_matches_energy_gradient(gm):
    rng = np.random.default_rng(2)
    phi = gm.urdf_params()
    eps = 1e-6
    for _ in range(15):
        q = rng.uniform(-1.5, 1.5, gm.n)
        g_model = gm.torque(q, phi)
        g_fd = np.zeros(gm.n)
        for j in range(gm.n):
            qp = q.copy(); qp[j] += eps
            qm = q.copy(); qm[j] -= eps
            g_fd[j] = (_numeric_potential(gm, qp) - _numeric_potential(gm, qm)) / (2 * eps)
        assert np.max(np.abs(g_model - g_fd)) < 1e-5


def test_params_roundtrip(gm):
    phi = gm.urdf_params()
    links = gm.params_to_links(phi)
    assert len(links) == gm.n
    # link1 mass/com from the URDF
    assert abs(links[0]['mass'] - 1.2668) < 1e-6
    assert np.allclose(links[0]['com'], [-0.0043321, 4.5056e-05, 0.19994], atol=1e-6)
