# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
"""Offline gravity-parameter identification.

Reads a CSV of static samples (q1..qN, tau1..tauN) produced by `collect`, builds
the gravity regressor from the URDF, and fits the per-link inertial parameters by
regularized least squares toward the URDF priors:

    min_phi || Y(q) phi - tau ||^2  +  || Gamma (phi - phi_urdf) ||^2

By default link masses are FIXED to their URDF values and only the first moments
(m*c, i.e. centre of mass) are identified. Gravity identifies first moments far
better than absolute masses, and fixing masses guarantees a physical, stable
result for the KDL-based controller. Pass --free-masses to identify all 4n
parameters with Tikhonov regularization instead.

    ros2 run armv7_dyn_ident identify --ros-args -p csv:=/tmp/armv7_gravity.csv

(or invoked directly:  python3 -m armv7_dyn_ident.identify --csv ... )
"""
from __future__ import annotations

import argparse
import datetime
import sys
from typing import List

import numpy as np
import yaml

from .gravity_model import GravityModel
from .urdf_model import parse_urdf_file, serial_chain


def _default_urdf() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return f'{get_package_share_directory("armv7_description")}/urdf/armv7.urdf'
    except Exception:
        return ''


def load_csv(path: str, n: int):
    raw = np.genfromtxt(path, delimiter=',', names=True)
    cols = raw.dtype.names
    qcols = [c for c in cols if c.startswith('q')]
    tcols = [c for c in cols if c.startswith('tau')]
    if len(qcols) != n or len(tcols) != n:
        raise ValueError(f'CSV has {len(qcols)} q-cols and {len(tcols)} tau-cols, '
                         f'expected {n} each')
    q = np.column_stack([raw[c] for c in qcols])
    tau = np.column_stack([raw[c] for c in tcols])
    return np.atleast_2d(q), np.atleast_2d(tau)


def build_stack(gm: GravityModel, q: np.ndarray):
    blocks = [gm.regressor(q[r]) for r in range(q.shape[0])]
    return np.vstack(blocks)                       # (N*n, 4n)


def identify(gm: GravityModel, q: np.ndarray, tau: np.ndarray,
             reg: float, free_masses: bool, joint_sign: np.ndarray):
    n = gm.n
    A = build_stack(gm, q)                         # (N*n, 4n)
    b = (tau * joint_sign[None, :]).reshape(-1)    # (N*n,)
    phi0 = gm.urdf_params()
    mass_cols = np.arange(0, 4 * n, 4)
    moment_cols = np.array([c for c in range(4 * n) if c not in mass_cols])

    if free_masses:
        # full Tikhonov toward prior; weight masses heavily to stay physical
        w = np.full(4 * n, reg)
        w[mass_cols] = reg * 100.0
        G = np.diag(w)
        lhs = A.T @ A + G.T @ G
        rhs = A.T @ b + (G.T @ G) @ phi0
        phi = np.linalg.solve(lhs, rhs)
        cond = np.linalg.cond(lhs)
    else:
        m_known = phi0[mass_cols]
        b_adj = b - A[:, mass_cols] @ m_known
        Ared = A[:, moment_cols]
        h0 = phi0[moment_cols]
        lhs = Ared.T @ Ared + reg * np.eye(Ared.shape[1])
        rhs = Ared.T @ b_adj + reg * h0
        h = np.linalg.solve(lhs, rhs)
        phi = phi0.copy()
        phi[moment_cols] = h
        # conditioning of the regularized system actually solved (finite); the
        # raw gravity regressor is rank-deficient by nature, hence the reg term.
        cond = np.linalg.cond(lhs)

    def rms(p):
        res = (A @ p - b).reshape(-1, n)
        return np.sqrt(np.mean(res ** 2, axis=0))

    return phi, rms(phi0), rms(phi), cond


def write_yaml(path: str, gm: GravityModel, phi, rms_before, rms_after,
               cond, args, n_samples):
    links = gm.params_to_links(phi)
    doc = {
        'identified_dynamics': {
            'joints': gm.joint_names,
            'gravity_axis': [float(x) for x in gm._gravity_axis],
            'links': [
                {'name': l['name'],
                 'mass': round(l['mass'], 6),
                 'com': [round(c, 6) for c in l['com']]}
                for l in links
            ],
            'meta': {
                'n_samples': int(n_samples),
                'fix_masses': not args.free_masses,
                'reg': args.reg,
                'rms_before_nm': [round(float(v), 4) for v in rms_before],
                'rms_after_nm': [round(float(v), 4) for v in rms_after],
                'regressor_cond': round(float(cond), 1),
                'source_csv': args.csv,
                'date': datetime.date.today().isoformat(),
            },
        }
    }
    with open(path, 'w') as f:
        yaml.safe_dump(doc, f, sort_keys=False, default_flow_style=None)


def _parse_args(argv: List[str]):
    p = argparse.ArgumentParser(description='armv7 gravity-parameter identification')
    p.add_argument('--csv', required=True, help='samples CSV from `collect`')
    p.add_argument('--urdf', default=_default_urdf(), help='URDF path')
    p.add_argument('--out', default='identified_params.yaml')
    p.add_argument('--reg', type=float, default=1e-3, help='regularization weight')
    p.add_argument('--free-masses', action='store_true',
                   help='identify masses too (default: fix to URDF)')
    p.add_argument('--joint-sign', type=float, nargs='+', default=None,
                   help='per-joint torque sign (+1/-1) if hardware convention differs')
    p.add_argument('--plot', action='store_true', help='show measured vs predicted')
    return p.parse_args(argv)


def main(argv=None):
    # tolerate ros2 run passing --ros-args ...
    argv = sys.argv[1:] if argv is None else argv
    if '--ros-args' in argv:
        argv = _from_ros_args(argv)
    args = _parse_args(argv)
    if not args.urdf:
        print('ERROR: no URDF found; pass --urdf', file=sys.stderr)
        return 1

    links, joints = parse_urdf_file(args.urdf)
    chain = serial_chain(links, joints)
    gm = GravityModel(chain)
    n = gm.n

    q, tau = load_csv(args.csv, n)
    sign = (np.ones(n) if args.joint_sign is None
            else np.asarray(args.joint_sign, dtype=float))
    if len(sign) != n:
        print(f'ERROR: --joint-sign needs {n} values', file=sys.stderr)
        return 1

    phi, rms_before, rms_after, cond = identify(
        gm, q, tau, args.reg, args.free_masses, sign)

    print(f'samples: {q.shape[0]}   regressor cond: {cond:.1f}')
    print('per-joint torque RMS residual (Nm):')
    for j, name in enumerate(gm.joint_names):
        print(f'  {name}:  urdf {rms_before[j]:7.3f}  ->  identified {rms_after[j]:7.3f}')
    print(f'mean: urdf {rms_before.mean():.3f} -> identified {rms_after.mean():.3f}')

    write_yaml(args.out, gm, phi, rms_before, rms_after, cond, args, q.shape[0])
    print(f'wrote {args.out}')

    if args.plot:
        _plot(gm, q, tau * sign[None, :], phi)
    return 0


def _from_ros_args(argv):
    """Extract -p key:=value pairs that ros2 run forwards into argparse flags."""
    out = []
    it = iter(argv)
    params = {}
    for tok in it:
        if tok in ('--ros-args', '-r'):
            continue
        if tok == '-p':
            kv = next(it, '')
            if ':=' in kv:
                k, v = kv.split(':=', 1)
                params[k] = v
    for k, v in params.items():
        out.append(f'--{k.replace("_", "-")}')
        out.append(v)
    return out


def _plot(gm, q, tau, phi):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print('matplotlib not installed; skipping plot', file=sys.stderr)
        return
    pred = np.vstack([gm.torque(q[r], phi) for r in range(q.shape[0])])
    fig, axes = plt.subplots(gm.n, 1, figsize=(8, 2 * gm.n), sharex=True)
    for j in range(gm.n):
        axes[j].plot(tau[:, j], label='measured')
        axes[j].plot(pred[:, j], '--', label='predicted')
        axes[j].set_ylabel(gm.joint_names[j])
    axes[0].legend()
    axes[-1].set_xlabel('sample')
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    sys.exit(main())
