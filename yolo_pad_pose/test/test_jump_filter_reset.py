"""Unit tests for the jump_filter_check helper in pad_waypoint_supervisor."""
import math

from yolo_pad_pose.pad_waypoint_supervisor import jump_filter_check


class TestJumpFilterCheck:
    """Tests for the pure jump_filter_check helper function."""

    # ── No anchor yet ──────────────────────────────────────────────────────────

    def test_accept_when_no_anchor(self):
        """Detection is always accepted when there is no existing anchor."""
        result = jump_filter_check(None, None, 1.0, 1.0, 2.0, 0, 10)
        assert result == 'accept'

    def test_accept_when_no_anchor_ignores_reset_count(self):
        """Even a high consecutive-reject count is irrelevant when anchor is absent."""
        result = jump_filter_check(None, None, 5.0, 5.0, 2.0, 99, 10)
        assert result == 'accept'

    # ── Small jump ────────────────────────────────────────────────────────────

    def test_accept_small_jump(self):
        """Detection within max_jump_m is accepted."""
        result = jump_filter_check(0.0, 0.0, 1.0, 1.0, 2.0, 0, 10)
        assert result == 'accept'

    def test_accept_exact_threshold(self):
        """Detection exactly at max_jump_m is accepted (≤ comparison)."""
        result = jump_filter_check(0.0, 0.0, 2.0, 0.0, 2.0, 0, 10)
        assert result == 'accept'

    # ── Large jump, below reset threshold ─────────────────────────────────────

    def test_reject_large_jump_first(self):
        """First large jump is rejected when reset count not reached."""
        result = jump_filter_check(0.0, 0.0, 5.0, 0.0, 2.0, 0, 10)
        assert result == 'reject'

    def test_reject_large_jump_below_reset(self):
        """Jump rejected when consecutive_rejects + 1 < jump_reject_reset_count."""
        result = jump_filter_check(0.0, 0.0, 5.0, 0.0, 2.0, 8, 10)
        assert result == 'reject'

    # ── Large jump, at reset threshold ────────────────────────────────────────

    def test_reset_at_threshold(self):
        """Reset returned when consecutive_rejects + 1 == jump_reject_reset_count."""
        result = jump_filter_check(0.0, 0.0, 5.0, 0.0, 2.0, 9, 10)
        assert result == 'reset'

    def test_reset_above_threshold(self):
        """Reset returned when consecutive_rejects + 1 > jump_reject_reset_count."""
        result = jump_filter_check(0.0, 0.0, 5.0, 0.0, 2.0, 15, 10)
        assert result == 'reset'

    # ── Auto-reset disabled ───────────────────────────────────────────────────

    def test_reject_when_reset_count_zero(self):
        """Setting jump_reject_reset_count=0 disables auto-reset (always reject)."""
        result = jump_filter_check(0.0, 0.0, 5.0, 0.0, 2.0, 999, 0)
        assert result == 'reject'

    # ── Geometry check ────────────────────────────────────────────────────────

    def test_jump_uses_hypot(self):
        """Jump distance uses Euclidean (hypot) of right and front deltas."""
        # Anchor at (3, 4); new at (0, 0). Jump = hypot(3, 4) = 5.
        result = jump_filter_check(3.0, 4.0, 0.0, 0.0, 4.9, 0, 10)
        assert result == 'reject'
        result = jump_filter_check(3.0, 4.0, 0.0, 0.0, 5.1, 0, 10)
        assert result == 'accept'

    def test_jump_threshold_boundary(self):
        """Boundary: jump just above max_jump_m is rejected."""
        eps = 1e-9
        jump = 2.0 + eps
        result = jump_filter_check(0.0, 0.0, jump, 0.0, 2.0, 0, 10)
        assert result == 'reject'

    # ── Consecutive counter ───────────────────────────────────────────────────

    def test_each_consecutive_reject_increments(self):
        """Outcomes for consecutive_rejects from 0 to reset_count-1 are all 'reject'."""
        reset_count = 5
        for i in range(reset_count - 1):
            result = jump_filter_check(0.0, 0.0, 5.0, 0.0, 2.0, i, reset_count)
            assert result == 'reject', f'Expected reject at consecutive_rejects={i}'

    def test_reset_triggers_exactly_at_count(self):
        """The Nth rejection (consecutive_rejects = N-1) triggers a reset."""
        reset_count = 5
        result = jump_filter_check(0.0, 0.0, 5.0, 0.0, 2.0, reset_count - 1, reset_count)
        assert result == 'reset'

    # ── reset_count = 1 (aggressive reset) ───────────────────────────────────

    def test_reset_count_one(self):
        """jump_reject_reset_count=1 resets on the very first rejection."""
        result = jump_filter_check(0.0, 0.0, 5.0, 0.0, 2.0, 0, 1)
        assert result == 'reset'

    # ── Negative coordinates ──────────────────────────────────────────────────

    def test_negative_coordinates(self):
        """Jump is computed from signed differences; negative coords are fine."""
        # Anchor (-3, -4), new (0, 0): jump = 5.
        result = jump_filter_check(-3.0, -4.0, 0.0, 0.0, 4.5, 0, 10)
        assert result == 'reject'
        result = jump_filter_check(-3.0, -4.0, 0.0, 0.0, 5.5, 0, 10)
        assert result == 'accept'

    # ── math.hypot(0, 0) == 0 corner case ────────────────────────────────────

    def test_zero_jump(self):
        """Detection at same position as anchor is accepted."""
        result = jump_filter_check(1.5, 2.5, 1.5, 2.5, 2.0, 0, 10)
        assert result == 'accept'

    def test_return_value_is_string(self):
        """Return value is always one of the three documented strings."""
        valid = {'accept', 'reject', 'reset'}
        cases = [
            (None, None, 1.0, 1.0, 2.0, 0, 10),
            (0.0, 0.0, 1.0, 0.0, 2.0, 0, 10),
            (0.0, 0.0, 5.0, 0.0, 2.0, 0, 10),
            (0.0, 0.0, 5.0, 0.0, 2.0, 9, 10),
        ]
        for args in cases:
            assert jump_filter_check(*args) in valid

    def test_large_jump_with_max_jump_zero(self):
        """When max_jump_m=0 every non-zero jump is rejected (or reset if count reached)."""
        # Even the tiniest displacement triggers a jump.
        result = jump_filter_check(0.0, 0.0, 0.001, 0.0, 0.0, 0, 10)
        assert result == 'reject'

    def test_hypot_vs_component(self):
        """Confirm jump uses full Euclidean distance, not just one axis."""
        # Anchor (0, 0), new (1, 1): hypot = sqrt(2) ≈ 1.414
        result = jump_filter_check(0.0, 0.0, 1.0, 1.0, math.sqrt(2) + 0.01, 0, 10)
        assert result == 'accept'
        result = jump_filter_check(0.0, 0.0, 1.0, 1.0, math.sqrt(2) - 0.01, 0, 10)
        assert result == 'reject'
