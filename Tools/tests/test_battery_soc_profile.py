# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026 PX4 Development Team.

import importlib.util
from pathlib import Path
import unittest

SPEC = importlib.util.spec_from_file_location("battery_soc_profile", Path(__file__).parents[1] / "battery_soc_profile.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class BatterySocProfileTest(unittest.TestCase):
    def setUp(self):
        self.rows = [
            {"relaxed_cell_voltage_v": 3.3 + 0.9 * soc * soc, "discharged_mah": 4000 * (1 - soc)}
            for soc in (0, 0.25, 0.5, 0.75, 1)
        ]

    def test_interpolates_measured_checkpoints_in_either_row_order(self):
        curve = MODULE.make_profile(self.rows[::-1], 4000)
        self.assertEqual(len(curve), 41)
        self.assertEqual(curve[0], 3.3)
        self.assertEqual(curve[20], 3.525)
        self.assertEqual(curve[-1], 4.2)
        self.assertAlmostEqual(curve[21], 3.553125)

    def test_rejects_missing_endpoints_and_sparse_measurements(self):
        for rows in (self.rows[1:], self.rows[:-1], self.rows[:2]):
            with self.assertRaises(ValueError):
                MODULE.make_profile(rows, 4000)

    def test_rejects_nonmonotonic_voltage(self):
        self.rows[2]["relaxed_cell_voltage_v"] = 4.1
        with self.assertRaises(ValueError):
            MODULE.make_profile(self.rows, 4000)

    def test_rejects_duplicate_discharge(self):
        self.rows.append(self.rows[2])
        with self.assertRaises(ValueError):
            MODULE.make_profile(self.rows, 4000)

    def test_rejects_invalid_capacity_and_measurements(self):
        for capacity in (0, -1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                MODULE.make_profile(self.rows, capacity)
        self.rows[2]["discharged_mah"] = float("nan")
        with self.assertRaises(ValueError):
            MODULE.make_profile(self.rows, 4000)

    def test_loaded_voltage_column_is_not_accepted_as_relaxed_data(self):
        with self.assertRaises(KeyError):
            MODULE.make_profile([{"cell_voltage_v": 3.8, "discharged_mah": 0}], 4000)


if __name__ == "__main__":
    unittest.main()
