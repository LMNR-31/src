"""Unit tests for pure helpers in base_waypoint_publisher.

These tests do **not** require a running ROS 2 environment; all imports are
handled via the stubs in conftest.py.
"""
import math

import pytest

from yolo_pad_pose.base_waypoint_publisher import (
    BaseEntry,
    body_to_world,
    cluster_update,
    merge_radius_from_area,
)


# ---------------------------------------------------------------------------
# merge_radius_from_area
# ---------------------------------------------------------------------------


class TestMergeRadiusFromArea:
    """Tests for the merge_radius_from_area helper."""

    def test_default_area_2_25(self):
        """Default merge_area_m2=2.25 → radius ≈ 0.846 m."""
        r = merge_radius_from_area(2.25)
        expected = math.sqrt(2.25 / math.pi)
        assert abs(r - expected) < 1e-12

    def test_default_area_approximate_value(self):
        """Radius for area=2.25 is approximately 0.846 m."""
        r = merge_radius_from_area(2.25)
        assert abs(r - 0.846) < 0.001

    def test_zero_area(self):
        """Zero area returns radius 0."""
        assert merge_radius_from_area(0.0) == 0.0

    def test_negative_area(self):
        """Negative area returns radius 0."""
        assert merge_radius_from_area(-1.0) == 0.0

    def test_unit_circle_area(self):
        """Area of pi → radius exactly 1.0."""
        r = merge_radius_from_area(math.pi)
        assert abs(r - 1.0) < 1e-12

    def test_small_area(self):
        """Small positive area returns small positive radius."""
        r = merge_radius_from_area(0.01)
        assert r > 0.0
        assert r < 0.1

    def test_large_area(self):
        """Large area returns proportionally large radius."""
        r = merge_radius_from_area(100.0 * math.pi)
        assert abs(r - 10.0) < 1e-10

    def test_inverse_round_trip(self):
        """Area = pi * r^2 and radius = sqrt(area/pi) are inverses."""
        for r_in in [0.5, 1.0, 1.5, 2.0, 5.0]:
            area = math.pi * r_in ** 2
            r_out = merge_radius_from_area(area)
            assert abs(r_out - r_in) < 1e-10


# ---------------------------------------------------------------------------
# cluster_update
# ---------------------------------------------------------------------------


class TestClusterUpdate:
    """Tests for the cluster_update helper."""

    def _make_base(self, x, y, seen=1):
        return BaseEntry(x=x, y=y, seen_count=seen)

    # ── Empty list ────────────────────────────────────────────────────────────

    def test_add_to_empty_list(self):
        """First detection always adds a new base."""
        bases, action = cluster_update([], 1.0, 2.0, 0.846, 6)
        assert action == 'added'
        assert len(bases) == 1
        assert bases[0].x == pytest.approx(1.0)
        assert bases[0].y == pytest.approx(2.0)

    # ── Merge ────────────────────────────────────────────────────────────────

    def test_merge_within_radius(self):
        """Detection within merge_radius updates the nearest base."""
        bases = [self._make_base(0.0, 0.0)]
        bases, action = cluster_update(bases, 0.1, 0.1, 0.846, 6)
        assert action == 'merged'
        assert len(bases) == 1
        assert bases[0].seen_count == 2

    def test_merge_updates_ema(self):
        """EMA update moves the base estimate toward the new observation."""
        bases = [self._make_base(0.0, 0.0)]
        nx, ny = 1.0, 0.0
        bases, _ = cluster_update(bases, nx, ny, 2.0, 6)
        # EMA alpha = 0.3: new_x = 0.7*0 + 0.3*1.0 = 0.3
        assert bases[0].x == pytest.approx(0.3)
        assert bases[0].y == pytest.approx(0.0)

    def test_merge_exact_at_radius(self):
        """Detection exactly at merge_radius is merged (≤ boundary)."""
        merge_r = 0.846
        bases = [self._make_base(0.0, 0.0)]
        # Place detection exactly at merge_radius along X axis
        bases, action = cluster_update(bases, merge_r, 0.0, merge_r, 6)
        assert action == 'merged'

    def test_merge_picks_closest_base(self):
        """When multiple bases exist, the closest one is updated."""
        bases = [self._make_base(0.0, 0.0), self._make_base(10.0, 0.0)]
        bases, action = cluster_update(bases, 0.2, 0.0, 2.0, 6)
        assert action == 'merged'
        # First base (at 0,0) was the nearest; its x should have moved toward 0.2
        assert bases[0].x == pytest.approx(0.3 * 0.2)
        # Second base unchanged
        assert bases[1].x == pytest.approx(10.0)

    # ── Add ───────────────────────────────────────────────────────────────────

    def test_add_when_beyond_radius(self):
        """Detection farther than merge_radius adds a new base."""
        bases = [self._make_base(0.0, 0.0)]
        bases, action = cluster_update(bases, 2.0, 0.0, 0.846, 6)
        assert action == 'added'
        assert len(bases) == 2

    def test_add_up_to_max_bases(self):
        """Can add up to max_bases bases."""
        bases = []
        for i in range(6):
            bases, action = cluster_update(bases, float(i * 5), 0.0, 0.846, 6)
            assert action == 'added'
        assert len(bases) == 6

    # ── Discard ───────────────────────────────────────────────────────────────

    def test_discard_when_full(self):
        """New cluster beyond merge_radius is discarded when max_bases reached."""
        bases = [self._make_base(float(i * 5), 0.0) for i in range(6)]
        bases, action = cluster_update(bases, 100.0, 0.0, 0.846, 6)
        assert action == 'discarded'
        assert len(bases) == 6

    def test_discard_does_not_modify_list(self):
        """Discarded detection leaves the base list unchanged."""
        bases = [self._make_base(float(i * 5), 0.0) for i in range(6)]
        original_coords = [(b.x, b.y) for b in bases]
        bases, _ = cluster_update(bases, 100.0, 0.0, 0.846, 6)
        assert [(b.x, b.y) for b in bases] == original_coords

    def test_discard_does_not_replace_worse_base(self):
        """Even if a new cluster is "closer" to drone than existing bases, it is
        still discarded when the list is full and does not fall within merge_radius
        of any known base."""
        bases = [self._make_base(float(i * 5), 0.0) for i in range(6)]
        old_len = len(bases)
        bases, action = cluster_update(bases, 99.9, 0.0, 0.846, 6)
        assert action == 'discarded'
        assert len(bases) == old_len

    # ── Custom max_bases ──────────────────────────────────────────────────────

    def test_max_bases_one(self):
        """max_bases=1: second distinct detection is discarded."""
        bases = []
        bases, a1 = cluster_update(bases, 0.0, 0.0, 0.5, 1)
        assert a1 == 'added'
        bases, a2 = cluster_update(bases, 5.0, 5.0, 0.5, 1)
        assert a2 == 'discarded'

    def test_max_bases_three(self):
        """max_bases=3: fourth distinct detection is discarded."""
        bases = []
        for i in range(3):
            bases, _ = cluster_update(bases, float(i * 5), 0.0, 0.5, 3)
        assert len(bases) == 3
        bases, action = cluster_update(bases, 50.0, 0.0, 0.5, 3)
        assert action == 'discarded'

    # ── seen_count bookkeeping ────────────────────────────────────────────────

    def test_seen_count_increments_on_merge(self):
        """seen_count increases by 1 on each merge."""
        bases = [self._make_base(0.0, 0.0, seen=1)]
        for expected in range(2, 6):
            bases, _ = cluster_update(bases, 0.0, 0.0, 1.0, 6)
            assert bases[0].seen_count == expected

    def test_new_base_seen_count_is_one(self):
        """Freshly added base has seen_count == 1."""
        bases, _ = cluster_update([], 0.0, 0.0, 1.0, 6)
        assert bases[0].seen_count == 1


# ---------------------------------------------------------------------------
# body_to_world
# ---------------------------------------------------------------------------


class TestBodyToWorld:
    """Tests for the body_to_world projection helper."""

    def test_no_yaw_forward(self):
        """At yaw=0, front offset maps directly to +X world."""
        wx, wy = body_to_world(front=1.0, right=0.0, cur_x=0.0, cur_y=0.0, yaw=0.0)
        assert wx == pytest.approx(1.0)
        assert wy == pytest.approx(0.0, abs=1e-10)

    def test_no_yaw_right(self):
        """At yaw=0, right offset maps directly to -Y world (ENU)."""
        wx, wy = body_to_world(front=0.0, right=1.0, cur_x=0.0, cur_y=0.0, yaw=0.0)
        assert wx == pytest.approx(0.0, abs=1e-10)
        assert wy == pytest.approx(-1.0)

    def test_yaw_90_deg(self):
        """At yaw=90°, forward (front) maps to +Y world; right maps to +X world."""
        yaw = math.pi / 2.0
        wx, wy = body_to_world(front=1.0, right=0.0, cur_x=0.0, cur_y=0.0, yaw=yaw)
        assert wx == pytest.approx(0.0, abs=1e-10)
        assert wy == pytest.approx(1.0)

    def test_offset_added_to_position(self):
        """Current position is added to the projection result."""
        wx, wy = body_to_world(
            front=1.0, right=0.0, cur_x=3.0, cur_y=4.0, yaw=0.0
        )
        assert wx == pytest.approx(4.0)
        assert wy == pytest.approx(4.0)

    def test_zero_detection(self):
        """Zero detection maps to the current position."""
        wx, wy = body_to_world(
            front=0.0, right=0.0, cur_x=1.5, cur_y=2.5, yaw=1.2
        )
        assert wx == pytest.approx(1.5)
        assert wy == pytest.approx(2.5)


# ---------------------------------------------------------------------------
# Multi-base tolerant detection (replaces body-frame jump filter)
# ---------------------------------------------------------------------------


class TestMultiBaseDetection:
    """Tests that world-frame clustering supports concurrent multi-base discovery.

    The old body-frame jump filter rejected any detection whose body-frame
    position jumped more than ``max_jump_m`` (default 2.0 m) from the last
    accepted detection.  When YOLO switched between bases (typical jump ~3 m)
    the second base was silently dropped.

    With the jump filter removed, ``cluster_update`` is the only gating
    mechanism and correctly handles detections from different physical bases
    by creating separate cluster entries.
    """

    def _make_base(self, x, y, seen=1):
        return BaseEntry(x=x, y=y, seen_count=seen)

    def test_two_far_apart_detections_both_registered(self):
        """Two world-frame detections >2 m apart each become a separate base.

        This scenario previously caused the second detection to be dropped by
        the body-frame jump filter (jump ~3 m > max_jump_m=2 m).
        """
        bases = []
        # Simulate first detection projected to world frame.
        bases, a1 = cluster_update(bases, 0.0, 5.0, 0.846, 6)
        assert a1 == 'added'
        # Simulate second detection ~3 m away in world frame (YOLO switched pads).
        bases, a2 = cluster_update(bases, 3.0, 5.0, 0.846, 6)
        assert a2 == 'added'
        assert len(bases) == 2

    def test_six_distinct_bases_all_registered(self):
        """All six distinct bases are registered even when spaced >2 m apart."""
        bases = []
        positions = [(float(i * 3), 0.0) for i in range(6)]
        for x, y in positions:
            bases, action = cluster_update(bases, x, y, 0.846, 6)
            assert action == 'added'
        assert len(bases) == 6

    def test_seventh_base_discarded_when_full(self):
        """When the list is full (6 bases), a 7th distinct cluster is discarded."""
        bases = [self._make_base(float(i * 3), 0.0) for i in range(6)]
        bases, action = cluster_update(bases, 100.0, 0.0, 0.846, 6)
        assert action == 'discarded'
        assert len(bases) == 6

    def test_switch_then_return_to_first_base_merges(self):
        """After jumping to a second base and back, the original base is updated."""
        bases = []
        # First detection → base #1 at (0, 5)
        bases, _ = cluster_update(bases, 0.0, 5.0, 0.846, 6)
        # Switch to base #2 at (4, 5)
        bases, _ = cluster_update(bases, 4.0, 5.0, 0.846, 6)
        assert len(bases) == 2
        # Return to near base #1 → should merge, not add
        bases, action = cluster_update(bases, 0.2, 5.1, 0.846, 6)
        assert action == 'merged'
        assert len(bases) == 2
        assert bases[0].seen_count == 2

    def test_alternating_two_bases_both_update(self):
        """Alternating detections between two bases each increment their counts."""
        bases = []
        bases, _ = cluster_update(bases, 0.0, 0.0, 0.846, 6)   # base #1
        bases, _ = cluster_update(bases, 5.0, 0.0, 0.846, 6)   # base #2
        # Alternate 4 more times
        for _ in range(2):
            bases, _ = cluster_update(bases, 0.1, 0.0, 0.846, 6)   # → base #1
            bases, _ = cluster_update(bases, 4.9, 0.0, 0.846, 6)   # → base #2
        assert bases[0].seen_count == 3
        assert bases[1].seen_count == 3

    def test_merge_radius_separates_close_from_far(self):
        """Detection within merge_radius merges; just beyond adds a new base."""
        merge_r = 0.846
        bases = [self._make_base(0.0, 0.0)]
        # Within merge radius → merged
        bases, a1 = cluster_update(bases, merge_r * 0.9, 0.0, merge_r, 6)
        assert a1 == 'merged'
        assert len(bases) == 1
        # Beyond merge radius → added
        bases, a2 = cluster_update(bases, 5.0, 0.0, merge_r, 6)
        assert a2 == 'added'
        assert len(bases) == 2
