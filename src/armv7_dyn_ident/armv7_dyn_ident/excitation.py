# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
"""Excitation trajectory generators for dynamics identification.

Two paths:

* ``static_poses`` — a set of distinct static configurations. The collector
  drives to each, lets it settle, and records (q, tau). This is the path used
  for *gravity* identification: at rest tau == G(q), no acceleration term, so
  it is robust and needs no differentiation. THIS is what feeds the
  gravity-compensation controller.

* ``fourier_trajectory`` — a band-limited periodic trajectory (finite Fourier
  series) that starts and ends at rest. Provided for *full* dynamic
  identification (inertia + Coriolis + friction) in a future release; the
  current identify pipeline does not use it.
"""
from __future__ import annotations

from typing import List, Sequence, Tuple

import numpy as np


def static_poses(lower: Sequence[float], upper: Sequence[float],
                 count: int = 60, margin: float = 0.12,
                 seed: int = 0) -> List[List[float]]:
    """Random configurations inside the joint limits (minus a safety margin).

    Random sampling gives a well-conditioned gravity regressor with far fewer
    poses than a full grid (which would be count**n). The first pose is always
    all-zeros (a known, safe reference) and the rest are reproducible for a
    given seed. The CALLER is responsible for confirming the poses are
    collision-free before running them on real hardware.
    """
    lower = np.asarray(lower, dtype=float)
    upper = np.asarray(upper, dtype=float)
    n = len(lower)
    span = (upper - lower)
    lo = lower + margin * span
    hi = upper - margin * span
    rng = np.random.default_rng(seed)
    poses = [list(np.zeros(n))]
    for _ in range(max(0, count - 1)):
        poses.append(list(rng.uniform(lo, hi)))
    return poses


def fourier_trajectory(coeffs_a: np.ndarray, coeffs_b: np.ndarray,
                       q0: Sequence[float], w_f: float,
                       duration: float, dt: float
                       ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Finite Fourier series trajectory, periodic and at rest at t=0 and t=T.

    For each joint j:
        q_j(t) = q0_j + sum_l [ a_jl/(w_f l) sin(w_f l t) - b_jl/(w_f l) cos(w_f l t) ]
    with the constant term chosen so q_dot(0)=q_dot(T)=0 when T = 2*pi/w_f * k.

    coeffs_a, coeffs_b: shape (n_joints, n_harmonics).
    Returns (t, q, qd, qdd) with q* shaped (len(t), n_joints).
    """
    coeffs_a = np.atleast_2d(np.asarray(coeffs_a, dtype=float))
    coeffs_b = np.atleast_2d(np.asarray(coeffs_b, dtype=float))
    q0 = np.asarray(q0, dtype=float)
    n_joints, n_harm = coeffs_a.shape
    t = np.arange(0.0, duration + 1e-9, dt)
    q = np.tile(q0, (len(t), 1)).astype(float)
    qd = np.zeros((len(t), n_joints))
    qdd = np.zeros((len(t), n_joints))
    for l in range(1, n_harm + 1):
        wl = w_f * l
        a = coeffs_a[:, l - 1]
        b = coeffs_b[:, l - 1]
        s = np.sin(wl * t)[:, None]
        c = np.cos(wl * t)[:, None]
        q += (a / wl) * s - (b / wl) * c
        qd += a * c + b * s
        qdd += -a * wl * s + b * wl * c
    return t, q, qd, qdd


def random_fourier_coeffs(n_joints: int, n_harmonics: int = 5,
                          amplitude: float = 0.3, seed: int = 0):
    rng = np.random.default_rng(seed)
    a = rng.uniform(-amplitude, amplitude, (n_joints, n_harmonics))
    b = rng.uniform(-amplitude, amplitude, (n_joints, n_harmonics))
    return a, b
