#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026 PX4 Development Team.
"""Convert relaxed-voltage calibration checkpoints into PX4 SOC profile parameters."""

import argparse
import csv
import math
import sys


PROFILE_POINTS = 41


def make_profile(rows, capacity_mah):
    """Interpolate 0..100% SOC from relaxed observations, without endpoint extrapolation."""
    if not math.isfinite(capacity_mah) or not 0 < capacity_mah <= 100000:
        raise ValueError("Measured usable capacity must be between 0 and 100000 mAh")
    checkpoints = []
    for row in rows:
        voltage = float(row["relaxed_cell_voltage_v"])
        discharge = float(row["discharged_mah"])
        if not math.isfinite(voltage) or not 2.5 <= voltage <= 5:
            raise ValueError("Relaxed cell voltages must be finite and between 2.5 and 5 V")
        if not math.isfinite(discharge) or not 0 <= discharge <= capacity_mah:
            raise ValueError("Discharge must be finite and within measured usable capacity")
        checkpoints.append((1 - discharge / capacity_mah, voltage))
    checkpoints.sort()
    if len(checkpoints) < 5:
        raise ValueError("At least five relaxed checkpoints, including both endpoints, are required")
    if checkpoints[0][0] > 1e-6 or checkpoints[-1][0] < 1 - 1e-6:
        raise ValueError("Include relaxed empty and full checkpoints; endpoint extrapolation is prohibited")
    for previous, current in zip(checkpoints, checkpoints[1:]):
        if current[0] <= previous[0] or current[1] <= previous[1]:
            raise ValueError("Relaxed voltage must strictly increase with SOC, without duplicate checkpoints")
    profile = []
    segment = 0
    for i in range(PROFILE_POINTS):
        soc = i / (PROFILE_POINTS - 1)
        while segment < len(checkpoints) - 2 and soc > checkpoints[segment + 1][0]:
            segment += 1
        low, high = checkpoints[segment:segment + 2]
        fraction = (soc - low[0]) / (high[0] - low[0])
        profile.append(round(low[1] + fraction * (high[1] - low[1]), 6))
    if any(high <= low for low, high in zip(profile, profile[1:])):
        raise ValueError("Profile points are indistinguishable at parameter precision")
    return profile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", help="CSV with relaxed_cell_voltage_v and discharged_mah columns")
    parser.add_argument("--capacity-mah", type=float, required=True, help="Measured usable capacity, including rest consumption")
    parser.add_argument("--cells", type=int, required=True, choices=range(1, 17))
    parser.add_argument("--resistance", type=float, required=True, help="Measured per-cell resistance in ohms")
    parser.add_argument("--battery", type=int, default=1, choices=range(1, 4))
    args = parser.parse_args()
    try:
        if not math.isfinite(args.resistance) or not 0 <= args.resistance <= 0.2:
            raise ValueError("Per-cell resistance must be between 0 and 0.2 ohms")
        with open(args.csv, encoding="utf-8-sig", newline="") as source:
            profile = make_profile(csv.DictReader(source), args.capacity_mah)
    except (OSError, ValueError, KeyError) as error:
        parser.error(str(error))
    prefix = f"BAT{args.battery}"
    print(f"param set {prefix}_SOC_MODE 1")
    print(f"param set {prefix}_N_CELLS {args.cells}")
    print(f"param set {prefix}_CAPACITY {args.capacity_mah:.6f}")
    print(f"param set {prefix}_R_INTERNAL {args.resistance:.6f}")
    for i, voltage in enumerate(profile):
        print(f"param set {prefix}_SOC_V{i} {voltage:.6f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
