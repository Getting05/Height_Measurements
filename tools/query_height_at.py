#!/usr/bin/env python3
"""Query one world xy point from a generated heightmap."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

TOOLS_DIR = Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from local_height_sampler import LocalHeightSampler


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--heightmap", required=True, help="Path to global_heightmap_filled.npy")
    parser.add_argument("--metadata", required=True, help="Path to metadata.yaml")
    parser.add_argument("--x", type=float, required=True, help="world_x in metadata frame")
    parser.add_argument("--y", type=float, required=True, help="world_y in metadata frame")
    parser.add_argument("--bilinear", action="store_true", help="Use bilinear interpolation")
    parser.add_argument("--default_height", type=float, default=0.0, help="Fallback height")
    args = parser.parse_args()

    sampler = LocalHeightSampler(args.heightmap, args.metadata, default_height=args.default_height)
    row, col = sampler.world_to_grid(args.x, args.y)
    row_i = int(row)
    col_i = int(col)
    in_bounds = 0 <= row_i < sampler.height and 0 <= col_i < sampler.width
    terrain_z = (
        sampler.query_bilinear(args.x, args.y)
        if args.bilinear
        else sampler.query_nearest(args.x, args.y)
    )
    print(f"frame_id: {sampler.frame_id}")
    print(f"world_x: {args.x:.6f}")
    print(f"world_y: {args.y:.6f}")
    print(f"row: {row_i}")
    print(f"col: {col_i}")
    print(f"in_bounds: {str(in_bounds).lower()}")
    print(f"terrain_z: {float(terrain_z):.6f}")


if __name__ == "__main__":
    main()
