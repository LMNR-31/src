"""Unit tests for the range_filter_check helper in yolo_pad_pose_ros2."""
import math

from yolo_pad_pose.yolo_pad_pose_ros2 import range_filter_check


class TestRangeFilterCheck:
    """Tests for the pure range_filter_check helper function."""

    # ── Filter disabled (max_range_m <= 0) ────────────────────────────────────

    def test_disabled_when_zero(self):
        """Filter is disabled when max_range_m == 0."""
        assert range_filter_check(100.0, 100.0, 0.0) is True

    def test_disabled_when_negative(self):
        """Filter is disabled when max_range_m is negative."""
        assert range_filter_check(100.0, 100.0, -1.0) is True

    # ── Within range ──────────────────────────────────────────────────────────

    def test_accept_origin(self):
        """Detection at origin (0, 0) is always accepted."""
        assert range_filter_check(0.0, 0.0, 5.0) is True

    def test_accept_small_range(self):
        """Detection well within max range is accepted."""
        assert range_filter_check(1.0, 1.0, 5.0) is True

    def test_accept_exact_threshold(self):
        """Detection exactly at max_range_m is accepted (<=)."""
        assert range_filter_check(3.0, 4.0, 5.0) is True  # hypot(3,4)=5

    # ── Beyond range ──────────────────────────────────────────────────────────

    def test_reject_beyond_range(self):
        """Detection beyond max_range_m is rejected."""
        assert range_filter_check(3.0, 4.0, 4.9) is False  # hypot(3,4)=5 > 4.9

    def test_reject_just_above_threshold(self):
        """Detection just above threshold is rejected."""
        eps = 1e-9
        assert range_filter_check(5.0 + eps, 0.0, 5.0) is False

    # ── Geometry ──────────────────────────────────────────────────────────────

    def test_uses_euclidean_distance(self):
        """Range is Euclidean hypot(right, front), not component-wise."""
        # right=3, front=4 → range=5; max=5 → accept
        assert range_filter_check(3.0, 4.0, 5.0) is True
        # max=4.9 → reject
        assert range_filter_check(3.0, 4.0, 4.9) is False

    def test_negative_coordinates(self):
        """Negative right/front values use absolute distance."""
        # hypot(-3, -4) == 5
        assert range_filter_check(-3.0, -4.0, 5.0) is True
        assert range_filter_check(-3.0, -4.0, 4.9) is False

    def test_only_right_component(self):
        """Works correctly with front=0."""
        assert range_filter_check(4.0, 0.0, 4.0) is True
        assert range_filter_check(4.1, 0.0, 4.0) is False

    def test_only_front_component(self):
        """Works correctly with right=0."""
        assert range_filter_check(0.0, 4.0, 4.0) is True
        assert range_filter_check(0.0, 4.1, 4.0) is False

    # ── Representative acceptance criterion value ─────────────────────────────

    def test_max_4_rejects_475m(self):
        """With max_base_range_m=4.0 a detection at ~4.75m is rejected."""
        # Values from the problem statement: right≈3.0, front≈3.7 → range≈4.75
        right, front = 3.0, 3.7
        _range = math.hypot(right, front)
        assert _range > 4.0
        assert range_filter_check(right, front, 4.0) is False

    def test_max_6_accepts_475m(self):
        """Default max_base_range_m=6.0 does not suppress the same detection."""
        assert range_filter_check(3.0, 3.7, 6.0) is True
