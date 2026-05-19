# Copyright 2026 TianFeiF
# SPDX-License-Identifier: Apache-2.0
"""Unit test for the bbox classifier (pure-Python, no ROS needed)."""
from __future__ import annotations

import sys
import types
from pathlib import Path


def _stub_rclpy() -> None:
    """Inject a minimal rclpy / tf2_ros / std_msgs / std_srvs stub.

    Lets us import workspace_bbox_node without bringing up rclpy. We only need
    the `_classify` method, so stubbing constructors is enough.
    """
    for mod_name in ['rclpy', 'rclpy.node', 'rclpy.time', 'tf2_ros',
                     'std_msgs', 'std_msgs.msg', 'std_srvs', 'std_srvs.srv']:
        if mod_name not in sys.modules:
            sys.modules[mod_name] = types.ModuleType(mod_name)

    # rclpy.node.Node placeholder — workspace_bbox_node subclasses it
    rclpy_node = sys.modules['rclpy.node']
    class _Node:  # noqa: D401
        def __init__(self, *_a, **_kw): pass
        def declare_parameter(self, *_a, **_kw): return _DummyParam()
        def get_parameter(self, *_a, **_kw): return _DummyParam()
        def create_publisher(self, *_a, **_kw): return None
        def create_client(self, *_a, **_kw): return None
        def create_timer(self, *_a, **_kw): return None
        def get_logger(self): return _Logger()
    rclpy_node.Node = _Node

    class _DummyParam:
        value = 0.0
    class _Logger:
        def info(self, *_a, **_kw): pass
        def warn(self, *_a, **_kw): pass
        def error(self, *_a, **_kw): pass

    # tf2_ros.TransformException stub
    sys.modules['tf2_ros'].TransformException = type('TransformException', (Exception,), {})
    sys.modules['tf2_ros'].Buffer = lambda: None
    sys.modules['tf2_ros'].TransformListener = lambda *_a, **_kw: None

    # std_msgs.msg.{Bool,String}
    msg_mod = sys.modules['std_msgs.msg']
    msg_mod.Bool = type('Bool', (), {'__init__': lambda self, data=False: setattr(self, 'data', data)})
    msg_mod.String = type('String', (), {'__init__': lambda self, data='': setattr(self, 'data', data)})

    # std_srvs.srv.Trigger
    srv_mod = sys.modules['std_srvs.srv']
    srv_mod.Trigger = type('Trigger', (), {'Request': type('Req', (), {'__init__': lambda self: None})})


def test_bbox_classify_basic():
    _stub_rclpy()
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'armv7_safety'))
    from workspace_bbox_node import WorkspaceBboxNode

    node = WorkspaceBboxNode.__new__(WorkspaceBboxNode)   # skip __init__
    node._bounds = (-0.5, 0.5, -0.5, 0.5, 0.0, 1.0)
    node._margin = 0.05

    assert node._classify(0.0, 0.0, 0.5) == 'ok'
    assert node._classify(0.48, 0.0, 0.5) == 'warning'    # within 0.05 of x face
    assert node._classify(0.0, 0.0, 1.05) == 'out_of_bounds'
    assert node._classify(-0.6, 0.0, 0.5) == 'out_of_bounds'
    assert node._classify(0.0, 0.0, 0.01) == 'warning'    # close to floor
