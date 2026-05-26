# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
"""Symbolic gravity model + linear regressor for a serial revolute chain.

Gravity is the only term that survives at static equilibrium (q_dot = q_ddot = 0),
so the measured joint torque there is exactly the gravity torque

    tau = G(q) = dU/dq ,   U = sum_i m_i * g * z_ci(q)

The potential energy is *linear* in the per-link inertial parameters
(m_i, h_i = m_i * c_i, the first moment), so

    G(q) = Y(q) . phi ,    phi = [m_1, hx_1, hy_1, hz_1, ..., m_n, hx_n, hy_n, hz_n]

This module builds Y(q) and G(q) symbolically (SymPy) once and lambdifies them to
fast NumPy callables. The full inertia tensor does not enter gravity, which is why
static identification only ever recovers mass + centre of mass — exactly what a
gravity-compensation controller needs.
"""
from __future__ import annotations

from typing import List, Sequence

import numpy as np
import sympy as sp

from .urdf_model import Segment

GRAVITY = 9.80665


def _rpy_matrix(roll, pitch, yaw):
    """URDF fixed-axis rpy -> Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = sp.cos(roll), sp.sin(roll)
    cp, sp_ = sp.cos(pitch), sp.sin(pitch)
    cy, sy = sp.cos(yaw), sp.sin(yaw)
    rx = sp.Matrix([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = sp.Matrix([[cp, 0, sp_], [0, 1, 0], [-sp_, 0, cp]])
    rz = sp.Matrix([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz * ry * rx


def _axis_rotation(axis: Sequence[float], angle):
    """Rodrigues rotation about a constant unit axis by symbolic angle."""
    ax = np.asarray(axis, dtype=float)
    ax = ax / np.linalg.norm(ax)
    x, y, z = float(ax[0]), float(ax[1]), float(ax[2])
    c, s = sp.cos(angle), sp.sin(angle)
    v = 1 - c
    return sp.Matrix([
        [x * x * v + c,     x * y * v - z * s, x * z * v + y * s],
        [x * y * v + z * s, y * y * v + c,     y * z * v - x * s],
        [x * z * v - y * s, y * z * v + x * s, z * z * v + c],
    ])


class GravityModel:
    """Linear gravity regressor for a serial chain of revolute Segments."""

    def __init__(self, chain: List[Segment],
                 gravity_axis: Sequence[float] = (0.0, 0.0, -1.0),
                 g: float = GRAVITY):
        self.chain = chain
        self.n = len(chain)
        self.joint_names = [seg.joint.name for seg in chain]
        self.link_names = [seg.link.name for seg in chain]
        self._g = g
        self._gravity_axis = np.asarray(gravity_axis, dtype=float)
        self._build()

    def _build(self):
        n = self.n
        q = sp.symbols(f'q0:{n}', real=True)
        params = []
        for i in range(n):
            params += [sp.Symbol(f'm{i}'), sp.Symbol(f'hx{i}'),
                       sp.Symbol(f'hy{i}'), sp.Symbol(f'hz{i}')]
        self._param_syms = params

        g_vec = sp.Matrix([self._g * float(a) for a in self._gravity_axis])

        T = sp.eye(4)
        U = sp.Integer(0)
        for i, seg in enumerate(self.chain):
            j = seg.joint
            t_origin = sp.eye(4)
            t_origin[:3, :3] = _rpy_matrix(*j.rpy)
            t_origin[0, 3], t_origin[1, 3], t_origin[2, 3] = j.xyz
            t_rot = sp.eye(4)
            t_rot[:3, :3] = _axis_rotation(j.axis, q[i])
            T = T * t_origin * t_rot

            rot = T[:3, :3]
            origin = T[:3, 3]
            m = params[4 * i]
            h = sp.Matrix(params[4 * i + 1:4 * i + 4])
            # m_i * p_ci = m_i * origin + R * h_i  (linear in [m, h])
            m_p = m * origin + rot * h
            U += -(g_vec.T * m_p)[0]

        g_torque = sp.Matrix([sp.diff(U, qi) for qi in q])           # n x 1
        regressor = g_torque.jacobian(sp.Matrix(params))             # n x 4n

        # No simplify(): for a 7-DoF chain it can take minutes and lambdify is
        # perfectly happy with the raw trig expressions. cse=True shares the
        # repeated sin/cos sub-terms, cutting both build and eval time.
        self._torque_fn = sp.lambdify(q, g_torque, 'numpy', cse=True)
        self._regressor_fn = sp.lambdify(q, regressor, 'numpy', cse=True)

    # -- numeric API ------------------------------------------------------
    def regressor(self, q: Sequence[float]) -> np.ndarray:
        """Y(q): shape (n, 4n)."""
        Y = np.asarray(self._regressor_fn(*q), dtype=float)
        return Y.reshape(self.n, 4 * self.n)

    def torque(self, q: Sequence[float], phi: Sequence[float]) -> np.ndarray:
        """G(q) = Y(q) . phi: shape (n,)."""
        return self.regressor(q) @ np.asarray(phi, dtype=float)

    # -- parameter <-> link conversions ----------------------------------
    def urdf_params(self) -> np.ndarray:
        """Prior phi from URDF inertials: [m_i, m_i*cx, m_i*cy, m_i*cz]."""
        phi = np.zeros(4 * self.n)
        for i, seg in enumerate(self.chain):
            m = seg.link.mass
            c = np.asarray(seg.link.com, dtype=float)
            phi[4 * i] = m
            phi[4 * i + 1:4 * i + 4] = m * c
        return phi

    def params_to_links(self, phi: Sequence[float], min_mass: float = 1e-4):
        """phi -> [{name, mass, com:[x,y,z]} ...] for each moving link."""
        phi = np.asarray(phi, dtype=float)
        out = []
        for i, seg in enumerate(self.chain):
            m = phi[4 * i]
            h = phi[4 * i + 1:4 * i + 4]
            safe_m = m if abs(m) > min_mass else min_mass
            out.append({
                'name': seg.link.name,
                'mass': float(m),
                'com': [float(v) for v in (h / safe_m)],
            })
        return out


def build_from_urdf_file(path: str, **kw) -> GravityModel:
    from .urdf_model import parse_urdf_file, serial_chain
    links, joints = parse_urdf_file(path)
    chain = serial_chain(links, joints)
    return GravityModel(chain, **kw)


def build_from_urdf_string(xml_text: str, **kw) -> GravityModel:
    from .urdf_model import parse_urdf_string, serial_chain
    links, joints = parse_urdf_string(xml_text)
    chain = serial_chain(links, joints)
    return GravityModel(chain, **kw)
