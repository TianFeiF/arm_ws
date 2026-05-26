# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
import numpy as np

from armv7_dyn_ident.excitation import (
    fourier_trajectory,
    random_fourier_coeffs,
    static_poses,
)

LOWER = [-3.14, -1.57, -3.14, -1.57, -3.14, -1.57, -3.14]
UPPER = [3.14, 1.57, 3.14, 1.57, 3.14, 1.57, 3.14]


def test_static_poses_within_limits():
    poses = static_poses(LOWER, UPPER, count=50, margin=0.1, seed=0)
    assert len(poses) == 50
    assert np.allclose(poses[0], 0.0)  # first pose is the zero reference
    lo = np.array(LOWER); hi = np.array(UPPER)
    for p in poses:
        assert np.all(np.array(p) >= lo)
        assert np.all(np.array(p) <= hi)


def test_static_poses_reproducible():
    a = static_poses(LOWER, UPPER, count=20, seed=7)
    b = static_poses(LOWER, UPPER, count=20, seed=7)
    assert np.allclose(a, b)


def test_fourier_starts_and_ends_at_rest():
    n = 7
    w_f = 2 * np.pi / 10.0          # period 10 s
    a, b = random_fourier_coeffs(n, n_harmonics=4, amplitude=0.2, seed=1)
    t, q, qd, qdd = fourier_trajectory(a, b, q0=np.zeros(n),
                                       w_f=w_f, duration=10.0, dt=0.01)
    assert q.shape == (len(t), n)
    # one full period -> velocity returns to its start value
    assert np.allclose(qd[0], qd[-1], atol=1e-6)
