"""Tests for the dependency-free sim-to-real waypoint export."""

from __future__ import annotations

import copy
import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from export_transfer import export_document, generate_paths  # noqa: E402


ROOT = Path(__file__).resolve().parents[1]
DEFAULT = json.loads((ROOT / "config" / "transfer.example.json").read_text())


class TransferTest(unittest.TestCase):
    def test_default_offsets_and_orientation_match_sim_run(self) -> None:
        bands = generate_paths(DEFAULT)
        self.assertEqual(3, len(bands))
        first = bands[0]
        self.assertEqual("arm_r", first["planning_group"])
        self.assertEqual("end_effector_r_link", first["ee_frame"])
        targets = first["target_poses"]
        self.assertEqual(["approach_start", "near_start", "near_end", "return_start", "retreat"],
                         [target["phase"] for target in targets])
        self.assertEqual([0.395, -0.41, 1.0875], _rounded(targets[0]["ee_xyz"]))
        self.assertEqual([0.435, -0.41, 1.0875], _rounded(targets[1]["ee_xyz"]))
        self.assertEqual([0.435, -0.15, 1.0875], _rounded(targets[2]["ee_xyz"]))
        self.assertEqual([0.0, -0.707107, 0.0, 0.707107], _rounded(targets[0]["quaternion_xyzw"]))
        self.assertEqual([0.475, -0.41, 1.0875], _rounded(targets[0]["contact_xyz"]))
        self.assertEqual([0.515, -0.41, 1.0875], _rounded(targets[1]["contact_xyz"]))

    def test_translation_and_rotation_of_board_preserve_local_path(self) -> None:
        reference = generate_paths(DEFAULT)[0]["target_poses"]
        transformed = copy.deepcopy(DEFAULT)
        transformed["board"].update({
            "origin": [2.0, 3.0, 4.0],
            "normal": [0.0, -1.0, 0.0],
            "u": [1.0, 0.0, 0.0],
            "v": [0.0, 0.0, 1.0],
        })
        targets = generate_paths(transformed)[0]["target_poses"]
        # Distances and the tool's local contact offset do not depend on board pose.
        for before, after in zip(reference, targets):
            self.assertAlmostEqual(0.08, _distance(before["ee_xyz"], before["contact_xyz"]))
            self.assertAlmostEqual(0.08, _distance(after["ee_xyz"], after["contact_xyz"]))
        self.assertAlmostEqual(0.26, _distance(targets[1]["contact_xyz"], targets[2]["contact_xyz"]))
        self.assertEqual([0.5, -0.5, 0.5, 0.5], _rounded(targets[0]["quaternion_xyzw"]))
        # Contact lies 80 mm opposite the board normal from the EE, despite rotation.
        self.assertEqual([0.0, 0.08, 0.0], _rounded(_minus(
            targets[1]["contact_xyz"], targets[1]["ee_xyz"])))

    def test_nonorthogonal_and_nonfinite_frames_are_rejected(self) -> None:
        bad = copy.deepcopy(DEFAULT)
        bad["board"]["u"] = [1.0, 0.0, 0.0]
        with self.assertRaisesRegex(ValueError, "orthogonal"):
            generate_paths(bad)
        bad = copy.deepcopy(DEFAULT)
        bad["board"]["normal"] = [float("nan"), 0.0, 0.0]
        with self.assertRaisesRegex(ValueError, "finite"):
            generate_paths(bad)
        bad = copy.deepcopy(DEFAULT)
        bad["board"]["v"] = [0.0, 0.0, -1.0]
        with self.assertRaisesRegex(ValueError, "cross"):
            generate_paths(bad)

    def test_document_states_real_transfer_limitations(self) -> None:
        document = export_document(DEFAULT)
        self.assertIn("hypothetical", document["contract"]["adapter"])
        self.assertIn("no force", document["contract"]["contact"])
        self.assertIn("rejected", document["contract"]["collision_basis"])
        self.assertEqual(3, len(document["bands"]))

    def test_pad_coverage_and_board_bounds_are_checked(self) -> None:
        bad = copy.deepcopy(DEFAULT)
        bad['tool']['pad_height'] = .01
        with self.assertRaisesRegex(ValueError, 'gap'):
            generate_paths(bad)
        bad = copy.deepcopy(DEFAULT)
        bad['tool']['pad_width'] = .20
        with self.assertRaisesRegex(ValueError, 'width'):
            generate_paths(bad)
        band = generate_paths(DEFAULT)[0]
        self.assertEqual(.02, band['suggested_max_contact_speed_m_s'])
        self.assertEqual(.07, band['tool_footprint_m']['width'])


def _rounded(values):
    return [round(value, 6) for value in values]


def _distance(first, second):
    return sum((a - b) ** 2 for a, b in zip(first, second)) ** 0.5


def _minus(first, second):
    return [a - b for a, b in zip(first, second)]


if __name__ == "__main__":
    unittest.main()
