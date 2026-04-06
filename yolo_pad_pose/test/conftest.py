"""Pytest configuration for yolo_pad_pose unit tests.

Sets up sys.path and stubs out ROS 2 modules so that Python source files
under ``yolo_pad_pose/`` can be imported in a plain Python environment
(without a full ROS 2 / ament installation).
"""
import os
import sys
import types

# Make the package importable from the repo tree.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# ---------------------------------------------------------------------------
# Minimal stubs for ROS 2 / message packages.
# Only the module-level symbols that the supervisor imports are mocked;
# no Node is ever instantiated during unit tests.
# ---------------------------------------------------------------------------
for _mod in [
    'rclpy', 'rclpy.node', 'rclpy.qos',
    'geometry_msgs', 'geometry_msgs.msg',
    'nav_msgs', 'nav_msgs.msg',
    'std_msgs', 'std_msgs.msg',
]:
    if _mod not in sys.modules:
        sys.modules[_mod] = types.ModuleType(_mod)

sys.modules['rclpy.node'].Node = object

_qos = sys.modules['rclpy.qos']
_qos.QoSProfile = object
_qos.DurabilityPolicy = types.SimpleNamespace(TRANSIENT_LOCAL=None)
_qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE=None)

for _attr, _mod in [
    ('Pose', 'geometry_msgs.msg'),
    ('PoseArray', 'geometry_msgs.msg'),
    ('PointStamped', 'geometry_msgs.msg'),
    ('Odometry', 'nav_msgs.msg'),
    ('Int32', 'std_msgs.msg'),
]:
    setattr(sys.modules[_mod], _attr, object)
