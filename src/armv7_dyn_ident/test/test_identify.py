# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
"""Synthetic self-consistency: identify recovers a perturbed gravity field."""
import numpy as np

from armv7_dyn_ident import identify as I


def _make_samples(gm, phi_true, n_poses, noise, seed):
    rng = np.random.default_rng(seed)
    q = np.array([rng.uniform(-1.4, 1.4, gm.n) for _ in range(n_poses)])
    tau = np.array([gm.torque(q[r], phi_true) + rng.normal(0, noise, gm.n)
                    for r in range(n_poses)])
    return q, tau


def test_identify_improves_on_urdf_prior(gm):
    rng = np.random.default_rng(1)
    phi0 = gm.urdf_params()
    phi_true = phi0.copy()
    for i in range(gm.n):
        phi_true[4 * i + 1:4 * i + 4] += rng.normal(0, 0.01, 3)

    q, tau = _make_samples(gm, phi_true, n_poses=60, noise=0.02, seed=3)
    phi_id, rms_before, rms_after, cond = I.identify(
        gm, q, tau, reg=1e-4, free_masses=False, joint_sign=np.ones(gm.n))

    # holdout error vs the TRUE field must shrink markedly
    test = np.array([rng.uniform(-1.4, 1.4, gm.n) for _ in range(200)])
    err_prior = np.mean([np.abs(gm.torque(qq, phi0) - gm.torque(qq, phi_true)).mean()
                         for qq in test])
    err_id = np.mean([np.abs(gm.torque(qq, phi_id) - gm.torque(qq, phi_true)).mean()
                      for qq in test])
    assert err_id < 0.3 * err_prior
    assert rms_after.mean() < rms_before.mean()
    assert np.isfinite(cond)


def test_load_csv_roundtrip(gm, tmp_path):
    import csv
    p = tmp_path / 'samples.csv'
    rng = np.random.default_rng(0)
    q = rng.uniform(-1, 1, (5, gm.n))
    tau = rng.uniform(-1, 1, (5, gm.n))
    with open(p, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow([f'q{i+1}' for i in range(gm.n)] + [f'tau{i+1}' for i in range(gm.n)])
        for r in range(5):
            w.writerow(list(q[r]) + list(tau[r]))
    q2, tau2 = I.load_csv(str(p), gm.n)
    assert np.allclose(q, q2)
    assert np.allclose(tau, tau2)
