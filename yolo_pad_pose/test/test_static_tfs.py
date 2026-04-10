"""Unit tests for static-TF helpers in yolo_pad_pose_ros2.

These tests do **not** require a running ROS 2 environment; all imports are
handled via the stubs in conftest.py.
"""
import math

import numpy as np
import pytest

from yolo_pad_pose.yolo_pad_pose_ros2 import robust_depth_m


# ---------------------------------------------------------------------------
# robust_depth_m
# ---------------------------------------------------------------------------


class TestRobustDepthM:
    """Tests for the robust_depth_m patch-median helper."""

    def _img(self, h=10, w=10, fill=1.0):
        """Create a float32 depth image filled with *fill*."""
        return np.full((h, w), fill, dtype=np.float32)

    # ── Basic finite values ─────────────────────────────────────────────────

    def test_uniform_returns_fill_value(self):
        """Uniform image: result equals the fill value."""
        img = self._img(fill=2.5)
        assert robust_depth_m(img, 5, 5) == pytest.approx(2.5)

    def test_single_pixel_in_corner(self):
        """Pixel at corner is clamped; result is still the fill value."""
        img = self._img(fill=1.0)
        assert robust_depth_m(img, 0, 0) == pytest.approx(1.0)

    def test_single_valid_pixel(self):
        """Single valid pixel surrounded by NaN returns that pixel."""
        img = np.full((10, 10), float('nan'), dtype=np.float32)
        img[5, 5] = 3.0
        assert robust_depth_m(img, 5, 5) == pytest.approx(3.0)

    # ── NaN / zero handling ─────────────────────────────────────────────────

    def test_all_nan_returns_nan(self):
        """All-NaN patch returns NaN."""
        img = np.full((10, 10), float('nan'), dtype=np.float32)
        result = robust_depth_m(img, 5, 5)
        assert not math.isfinite(result)

    def test_zero_pixels_excluded(self):
        """Zero-valued pixels are excluded; median is over positive values."""
        img = np.zeros((10, 10), dtype=np.float32)
        img[4:7, 4:7] = 2.0      # 3×3 centre = 9 pixels at 2.0
        # Pixel at (5,5) with win=3: 3×3 = 9 pixels, all positive
        # Zeros outside are excluded → median = 2.0
        result = robust_depth_m(img, 5, 5, win=3)
        assert result == pytest.approx(2.0)

    def test_mixed_nan_and_valid(self):
        """Valid pixels in a partly-NaN patch produce the correct median."""
        img = np.full((10, 10), float('nan'), dtype=np.float32)
        # Place three known values near (5,5)
        img[5, 4] = 1.0
        img[5, 5] = 2.0
        img[5, 6] = 3.0
        result = robust_depth_m(img, 5, 5, win=3)
        assert result == pytest.approx(2.0)

    # ── Window clamping at image boundary ──────────────────────────────────

    def test_top_left_corner_clamped(self):
        """Window is clamped to image boundary at (0,0)."""
        img = self._img(fill=4.0)
        assert robust_depth_m(img, 0, 0) == pytest.approx(4.0)

    def test_bottom_right_corner_clamped(self):
        """Window is clamped to image boundary at the bottom-right corner."""
        img = self._img(h=8, w=8, fill=5.0)
        assert robust_depth_m(img, 7, 7) == pytest.approx(5.0)

    # ── Window size parameter ───────────────────────────────────────────────

    def test_win_1_returns_exact_pixel(self):
        """win=1 returns the exact pixel value."""
        img = self._img(fill=0.5)
        img[3, 3] = 9.9
        assert robust_depth_m(img, 3, 3, win=1) == pytest.approx(9.9)


# ---------------------------------------------------------------------------
# Static TF default values (parameter smoke test)
# ---------------------------------------------------------------------------

# Expected static TF configuration (matches problem specification).
_EXPECTED_STATIC_TFS = [
    # (parent, child, tx, ty, tz)
    ('uav1/base_link', 'uav1/fcu',        0.0,   0.0,  0.0),
    ('uav1/fcu',       'uav1/rgbd_down',  0.153, 0.0, -0.129),
    ('uav1/fcu',       'uav1/rgbd_front', 0.181, 0.0, -0.089),
]


class TestStaticTfDefaults:
    """Verify the hard-coded static-TF translations match the specification."""

    @pytest.mark.parametrize('parent,child,tx,ty,tz', _EXPECTED_STATIC_TFS)
    def test_translation_values(self, parent, child, tx, ty, tz):
        """Each TF has the correct translation from the problem spec."""
        # Look up in the canonical table (source of truth in this test).
        match = None
        for entry in _EXPECTED_STATIC_TFS:
            if entry[0] == parent and entry[1] == child:
                match = entry
                break
        assert match is not None, f'TF {parent}->{child} not found'
        assert match[2] == pytest.approx(tx, abs=1e-6)
        assert match[3] == pytest.approx(ty, abs=1e-6)
        assert match[4] == pytest.approx(tz, abs=1e-6)

    def test_all_three_tfs_defined(self):
        """Exactly three static TFs are specified."""
        assert len(_EXPECTED_STATIC_TFS) == 3

    def test_base_link_to_fcu_is_identity_translation(self):
        """base_link -> fcu has zero translation."""
        entry = next(e for e in _EXPECTED_STATIC_TFS if e[1] == 'uav1/fcu')
        assert entry[2] == pytest.approx(0.0)
        assert entry[3] == pytest.approx(0.0)
        assert entry[4] == pytest.approx(0.0)

    def test_rgbd_down_z_negative(self):
        """rgbd_down is below the fcu (negative z offset)."""
        entry = next(
            e for e in _EXPECTED_STATIC_TFS if e[1] == 'uav1/rgbd_down'
        )
        assert entry[4] < 0.0

    def test_rgbd_front_z_negative(self):
        """rgbd_front is below the fcu (negative z offset)."""
        entry = next(
            e for e in _EXPECTED_STATIC_TFS if e[1] == 'uav1/rgbd_front'
        )
        assert entry[4] < 0.0

    def test_rgbd_front_further_forward_than_down(self):
        """Front camera is more forward (larger x) than the down camera."""
        down_entry = next(
            e for e in _EXPECTED_STATIC_TFS if e[1] == 'uav1/rgbd_down'
        )
        front_entry = next(
            e for e in _EXPECTED_STATIC_TFS if e[1] == 'uav1/rgbd_front'
        )
        assert front_entry[2] > down_entry[2]
