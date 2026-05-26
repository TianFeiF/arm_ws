# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
"""Minimal URDF parser for dynamics identification.

Pulls just what the gravity model needs out of a URDF string/file: the serial
chain of revolute joints (origin, axis, limits) and each child link's inertial
mass + centre of mass. Deliberately dependency-free (xml.etree only) so it runs
in CI without urdfdom / pinocchio.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, List, Optional


def _floats(text: Optional[str], n: int, default):
    if text is None:
        return list(default)
    vals = [float(x) for x in text.split()]
    if len(vals) != n:
        raise ValueError(f'expected {n} floats, got {text!r}')
    return vals


@dataclass
class Joint:
    name: str
    jtype: str
    parent: str
    child: str
    xyz: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    rpy: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    axis: List[float] = field(default_factory=lambda: [0.0, 0.0, 1.0])
    lower: float = 0.0
    upper: float = 0.0
    effort: float = 0.0
    velocity: float = 0.0


@dataclass
class Link:
    name: str
    mass: float = 0.0
    com: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])


@dataclass
class Segment:
    """One moving joint plus the child link rigidly attached past it."""
    joint: Joint
    link: Link


_MOVING = ('revolute', 'continuous', 'prismatic')


def parse_urdf_string(xml_text: str):
    root = ET.fromstring(xml_text)

    links: Dict[str, Link] = {}
    for le in root.findall('link'):
        name = le.get('name')
        link = Link(name=name)
        inertial = le.find('inertial')
        if inertial is not None:
            mass_e = inertial.find('mass')
            if mass_e is not None:
                link.mass = float(mass_e.get('value', '0.0'))
            origin_e = inertial.find('origin')
            if origin_e is not None:
                link.com = _floats(origin_e.get('xyz'), 3, [0, 0, 0])
        links[name] = link

    joints: Dict[str, Joint] = {}
    for je in root.findall('joint'):
        j = Joint(
            name=je.get('name'),
            jtype=je.get('type'),
            parent=je.find('parent').get('link'),
            child=je.find('child').get('link'),
        )
        origin_e = je.find('origin')
        if origin_e is not None:
            j.xyz = _floats(origin_e.get('xyz'), 3, [0, 0, 0])
            j.rpy = _floats(origin_e.get('rpy'), 3, [0, 0, 0])
        axis_e = je.find('axis')
        if axis_e is not None:
            j.axis = _floats(axis_e.get('xyz'), 3, [0, 0, 1])
        limit_e = je.find('limit')
        if limit_e is not None:
            j.lower = float(limit_e.get('lower', '0.0'))
            j.upper = float(limit_e.get('upper', '0.0'))
            j.effort = float(limit_e.get('effort', '0.0'))
            j.velocity = float(limit_e.get('velocity', '0.0'))
        joints[j.name] = j

    return links, joints


def parse_urdf_file(path: str):
    with open(path, 'r') as f:
        return parse_urdf_string(f.read())


def _root_link(links, joints) -> str:
    children = {j.child for j in joints.values()}
    roots = [name for name in links if name not in children]
    if len(roots) != 1:
        raise ValueError(f'expected exactly one root link, found {roots}')
    return roots[0]


def serial_chain(links: Dict[str, Link], joints: Dict[str, Joint],
                 root: Optional[str] = None,
                 tip: Optional[str] = None) -> List[Segment]:
    """Ordered list of moving Segments from the root link outward.

    Fixed joints are folded into the preceding moving segment is NOT done here
    (this URDF is a clean serial revolute chain); fixed joints are skipped and
    a warning is the caller's job. For the armv7 arm every joint is revolute.
    """
    if root is None:
        root = _root_link(links, joints)

    by_parent: Dict[str, List[Joint]] = {}
    for j in joints.values():
        by_parent.setdefault(j.parent, []).append(j)

    chain: List[Segment] = []
    current = root
    while current in by_parent:
        outgoing = by_parent[current]
        # serial chain: take the (single) moving joint continuing the arm
        nxt = None
        for j in outgoing:
            if j.jtype in _MOVING:
                nxt = j
                break
        if nxt is None:
            break
        chain.append(Segment(joint=nxt, link=links[nxt.child]))
        current = nxt.child
        if tip is not None and current == tip:
            break
    return chain
