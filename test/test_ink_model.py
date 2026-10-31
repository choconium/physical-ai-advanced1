"""Unit tests for the standard-library-only geometric whiteboard model."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from ink_model import (  # noqa: E402
    DEFAULT_BOARD,
    InkPixel,
    apply_tool_sample,
    build_sdf,
    coverage_metrics,
    initial_pixels,
)


IDENTITY = (0.0, 0.0, 0.0, 1.0)


class InkModelTest(unittest.TestCase):
    def test_initial_ai_bitmap_is_bounded_and_reports_zero_coverage(self) -> None:
        pixels = initial_pixels()
        self.assertEqual(27, len(pixels))
        self.assertEqual(len({pixel.id for pixel in pixels}), len(pixels))
        self.assertTrue(all(DEFAULT_BOARD.contains_yz(pixel.y, pixel.z) for pixel in pixels))
        self.assertEqual(
            {"total_pixels": 27, "erased_pixels": 0, "residual_pixels": 27, "erasure_ratio": 0.0},
            coverage_metrics(pixels),
        )

    def test_disabled_wrong_depth_and_wrong_orientation_never_erase(self) -> None:
        pixels = initial_pixels()
        target = (DEFAULT_BOARD.surface_x, -0.34, 1.05)
        scenarios = (
            (target, IDENTITY, False),
            ((DEFAULT_BOARD.surface_x - 0.009, -0.34, 1.05), IDENTITY, True),
            ((DEFAULT_BOARD.surface_x + 0.004, -0.34, 1.05), IDENTITY, True),
            (target, (0.0, 0.0, 1.0, 0.0), True),  # local +x points world -x
        )
        for tool_xyz, orientation, enabled in scenarios:
            with self.subTest(tool_xyz=tool_xyz, orientation=orientation, enabled=enabled):
                output = apply_tool_sample(pixels, tool_xyz, orientation, enabled)
                self.assertEqual(0, coverage_metrics(output)["erased_pixels"])
                self.assertEqual(pixels, output)

    def test_eligible_contact_erases_only_cells_in_eraser_footprint(self) -> None:
        pixels = initial_pixels()
        output = apply_tool_sample(
            pixels,
            (DEFAULT_BOARD.surface_x, -0.34, 1.05),
            IDENTITY,
            True,
        )
        changed = [after for before, after in zip(pixels, output) if before.erased != after.erased]
        self.assertGreater(len(changed), 0)
        self.assertLess(len(changed), len(pixels))
        self.assertTrue(all(abs(pixel.y + 0.34) <= 0.035 for pixel in changed))
        self.assertTrue(all(abs(pixel.z - 1.05) <= 0.0225 for pixel in changed))
        metrics = coverage_metrics(output)
        self.assertEqual(len(changed), metrics["erased_pixels"])
        self.assertEqual(1.0 - len(changed) / len(pixels), metrics["residual_pixels"] / len(pixels))

    def test_sdf_has_collidable_board_and_independent_visual_only_ink(self) -> None:
        pixels = initial_pixels()
        sdf = build_sdf(DEFAULT_BOARD, pixels)
        self.assertIn('<model name="whiteboard">', sdf)
        self.assertIn('<collision name="board_collision">', sdf)
        self.assertIn('<size>0.020000 0.360000 0.300000</size>', sdf)
        self.assertEqual(len(pixels), sdf.count('<model name="ink_'))
        self.assertNotIn('collision name="ink_', sdf)
        self.assertIn('<pose>0.518000', sdf)

    def test_invalid_samples_are_rejected(self) -> None:
        pixels = [InkPixel("ink_test", -0.28, 1.05)]
        with self.assertRaises(ValueError):
            apply_tool_sample(pixels, (float("nan"), 0.0, 1.0), IDENTITY, True)
        with self.assertRaises(ValueError):
            apply_tool_sample(pixels, (0.52, -0.28, 1.05), (0.0, 0.0, 0.0, 0.0), True)


if __name__ == "__main__":
    unittest.main()
