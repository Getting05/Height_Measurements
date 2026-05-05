#!/usr/bin/env python3
"""Convert a PCD point cloud into a regular global heightmap."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import logging
import math
from pathlib import Path
import time
from typing import Tuple

import numpy as np
import yaml


LOGGER = logging.getLogger("pcd_to_global_heightmap")


def optional_float(value: str) -> float | None:
    if value.lower() in {"none", "off", "disable", "disabled"}:
        return None
    return float(value)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_pcd", nargs="?", help="Input .pcd path")
    parser.add_argument("--pcd", dest="pcd", help="Input .pcd path")
    parser.add_argument("--output_dir", required=True, help="Directory for generated heightmap files")
    parser.add_argument("--resolution", type=float, default=0.025, help="Heightmap resolution in meters")
    parser.add_argument("--voxel_size", type=float, default=0.025, help="Open3D voxel downsample size; <=0 disables")
    parser.add_argument("--z_mode", choices=["min", "max", "mean"], default="min", help="Per-cell z aggregation mode")
    parser.add_argument("--z_min", type=optional_float, default=None, help="Optional minimum z filter")
    parser.add_argument(
        "--z_max",
        type=optional_float,
        default=1.0,
        help="Optional maximum z filter applied before voxel downsampling; use 'none' to disable",
    )
    parser.add_argument("--frame_id", default="map", help="Frame id stored in metadata.yaml")
    parser.add_argument("--fill_iterations", type=int, default=20, help="Local mean NaN fill iterations")
    parser.add_argument("--fill_size", type=int, default=3, help="Odd filter size for local mean NaN fill")
    parser.add_argument("--skip_png", action="store_true", help="Skip matplotlib PNG previews")
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> Path:
    pcd_arg = args.pcd or args.input_pcd
    if not pcd_arg:
        raise ValueError("provide an input .pcd path as a positional argument or --pcd")
    pcd_path = Path(pcd_arg)
    if not pcd_path.exists():
        raise FileNotFoundError(f"PCD file does not exist: {pcd_path}")
    if not pcd_path.is_file():
        raise ValueError(f"PCD path is not a file: {pcd_path}")
    if args.resolution <= 0.0:
        raise ValueError("--resolution must be positive")
    if args.voxel_size < 0.0:
        raise ValueError("--voxel_size must be >= 0")
    if args.z_min is not None and args.z_max is not None and args.z_min > args.z_max:
        raise ValueError("--z_min must be <= --z_max")
    if args.fill_iterations < 0:
        raise ValueError("--fill_iterations must be >= 0")
    if args.fill_size < 1 or args.fill_size % 2 == 0:
        raise ValueError("--fill_size must be a positive odd integer")
    return pcd_path


@contextmanager
def timed_step(name: str):
    start = time.perf_counter()
    try:
        yield
    finally:
        LOGGER.info("%s took %.2f s", name, time.perf_counter() - start)


def load_points(pcd_path: Path, voxel_size: float, z_min: float | None, z_max: float | None) -> np.ndarray:
    try:
        import open3d as o3d
    except ImportError as exc:
        raise RuntimeError("open3d is required: python3 -m pip install open3d") from exc

    LOGGER.info("Reading point cloud: %s", pcd_path)
    pcd = o3d.io.read_point_cloud(str(pcd_path))
    if pcd.is_empty():
        raise RuntimeError(f"point cloud is empty: {pcd_path}")
    LOGGER.info("Read %d points", len(pcd.points))

    if z_min is not None or z_max is not None:
        points = np.asarray(pcd.points)
        mask = np.ones(points.shape[0], dtype=bool)
        if z_min is not None:
            mask &= points[:, 2] >= z_min
        if z_max is not None:
            mask &= points[:, 2] <= z_max
        kept = int(np.sum(mask))
        if kept == 0:
            raise RuntimeError("z prefilter removed all points")
        LOGGER.info(
            "Z prefilter before voxel downsample kept %d/%d points (z_min=%s, z_max=%s)",
            kept,
            points.shape[0],
            "none" if z_min is None else f"{z_min:.4f}",
            "none" if z_max is None else f"{z_max:.4f}",
        )
        if kept != points.shape[0]:
            keep_indices = np.flatnonzero(mask)
            pcd = pcd.select_by_index(keep_indices)
            del keep_indices
        del points, mask

    if voxel_size > 0.0:
        LOGGER.info("Voxel downsampling with voxel_size=%.4f m", voxel_size)
        pcd = pcd.voxel_down_sample(voxel_size)
        if pcd.is_empty():
            raise RuntimeError("point cloud became empty after voxel downsample")

    points = np.asarray(pcd.points, dtype=np.float64)
    if points.size == 0:
        raise RuntimeError("point cloud has no xyz points")
    LOGGER.info("Loaded %d points", points.shape[0])
    return points


def filter_points(points: np.ndarray, z_min: float | None, z_max: float | None) -> np.ndarray:
    if z_min is None and z_max is None:
        LOGGER.info("Using %d points after z filtering", points.shape[0])
        return points

    mask = np.ones(points.shape[0], dtype=bool)
    if z_min is not None:
        mask &= points[:, 2] >= z_min
    if z_max is not None:
        mask &= points[:, 2] <= z_max
    filtered = points[mask]
    if filtered.size == 0:
        raise RuntimeError("z filtering removed all points")
    LOGGER.info("Using %d points after z filtering", filtered.shape[0])
    return filtered


def points_to_heightmap(points: np.ndarray, resolution: float, z_mode: str) -> Tuple[np.ndarray, dict]:
    origin_x = float(np.min(points[:, 0]))
    origin_y = float(np.min(points[:, 1]))
    x_max = float(np.max(points[:, 0]))
    y_max = float(np.max(points[:, 1]))

    width = int(math.ceil((x_max - origin_x) / resolution)) + 1
    height = int(math.ceil((y_max - origin_y) / resolution)) + 1
    if width <= 0 or height <= 0:
        raise RuntimeError(f"invalid heightmap shape: height={height}, width={width}")
    cell_count = height * width
    LOGGER.info("Heightmap grid: shape=(%d, %d), cells=%d", height, width, cell_count)

    col = ((points[:, 0] - origin_x) / resolution).astype(np.int64)
    row = ((points[:, 1] - origin_y) / resolution).astype(np.int64)
    np.clip(col, 0, width - 1, out=col)
    np.clip(row, 0, height - 1, out=row)
    flat_index = row * width + col
    del col, row
    z = points[:, 2]

    if z_mode == "mean":
        counts = np.bincount(flat_index, minlength=cell_count)
        sums = np.bincount(flat_index, weights=z, minlength=cell_count)
        flat = np.full(cell_count, np.nan, dtype=np.float32)
        valid = counts > 0
        flat[valid] = (sums[valid] / counts[valid]).astype(np.float32)
    elif z_mode in {"min", "max"}:
        order = np.argsort(flat_index)
        sorted_index = flat_index[order]
        sorted_z = z[order]
        starts = np.empty(sorted_index.size, dtype=np.int64)
        starts[0] = 0
        group_count = int(np.count_nonzero(sorted_index[1:] != sorted_index[:-1])) + 1
        starts = starts[:group_count]
        starts[1:] = np.flatnonzero(sorted_index[1:] != sorted_index[:-1]) + 1
        cells = sorted_index[starts]
        if z_mode == "min":
            values = np.minimum.reduceat(sorted_z, starts)
        else:
            values = np.maximum.reduceat(sorted_z, starts)

        flat = np.full(cell_count, np.nan, dtype=np.float32)
        flat[cells] = values.astype(np.float32, copy=False)
    else:
        raise ValueError(f"unsupported z_mode: {z_mode}")

    height_map = flat.reshape(height, width)
    metadata = {
        "resolution": float(resolution),
        "origin_x": origin_x,
        "origin_y": origin_y,
        "width": int(width),
        "height": int(height),
        "z_mode": z_mode,
    }
    return height_map, metadata


def fill_nans_local_mean(raw: np.ndarray, max_iterations: int = 20, filter_size: int = 3) -> np.ndarray:
    """Fill NaNs by repeatedly copying local finite neighborhood means."""
    filled = raw.astype(np.float32, copy=True)
    if not np.any(np.isnan(filled)):
        return filled

    try:
        from scipy import ndimage
    except ImportError as exc:
        raise RuntimeError("scipy is required for NaN filling: python3 -m pip install scipy") from exc

    kernel = np.ones((filter_size, filter_size), dtype=np.float32)
    for iteration in range(max_iterations):
        nan_mask = np.isnan(filled)
        if not np.any(nan_mask):
            LOGGER.info("NaN fill completed after %d iterations", iteration)
            return filled

        finite = np.isfinite(filled)
        sums = ndimage.convolve(np.where(finite, filled, 0.0), kernel, mode="nearest")
        counts = ndimage.convolve(finite.astype(np.float32), kernel, mode="nearest")
        local_mean = np.full_like(filled, np.nan)
        np.divide(sums, counts, out=local_mean, where=counts > 0.0)

        replace = nan_mask & (counts > 0.0)
        if not np.any(replace):
            LOGGER.warning("NaN fill stalled after %d iterations", iteration)
            break
        filled[replace] = local_mean[replace]
        LOGGER.info("NaN fill iteration %d replaced %d cells", iteration + 1, int(np.sum(replace)))

    remaining = np.isnan(filled)
    if np.any(remaining):
        finite = filled[np.isfinite(filled)]
        fallback = float(np.mean(finite)) if finite.size else 0.0
        LOGGER.warning("Filling %d remaining NaNs with global fallback %.4f", int(np.sum(remaining)), fallback)
        filled[remaining] = fallback
    return filled


def save_png(path: Path, array: np.ndarray, title: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    LOGGER.info("Writing %s", path)
    fig, ax = plt.subplots(figsize=(10, 8))
    image = ax.imshow(np.ma.masked_invalid(array), origin="lower", interpolation="nearest")
    ax.set_title(title)
    ax.set_xlabel("col")
    ax.set_ylabel("row")
    fig.colorbar(image, ax=ax, label="z [m]")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    args = parse_args()
    pcd_path = validate_args(args)
    output_dir = Path(args.output_dir)
    if output_dir.resolve() == Path("/"):
        LOGGER.warning("--output_dir is '/', generated files will be written to the system root")
    output_dir.mkdir(parents=True, exist_ok=True)

    with timed_step("Load point cloud"):
        points = load_points(pcd_path, args.voxel_size, args.z_min, args.z_max)
    with timed_step("Build raw heightmap"):
        raw, metadata = points_to_heightmap(points, args.resolution, args.z_mode)
    metadata["frame_id"] = args.frame_id
    metadata["pcd_path"] = str(pcd_path)
    metadata["z_min"] = args.z_min
    metadata["z_max"] = args.z_max

    with timed_step("Fill NaNs"):
        filled = fill_nans_local_mean(raw, args.fill_iterations, args.fill_size)

    LOGGER.info(
        "Heightmap shape=(%d, %d), resolution=%.4f, origin=(%.4f, %.4f), raw_nan=%d",
        metadata["height"],
        metadata["width"],
        metadata["resolution"],
        metadata["origin_x"],
        metadata["origin_y"],
        int(np.sum(np.isnan(raw))),
    )

    with timed_step("Write NumPy/YAML outputs"):
        np.save(output_dir / "global_heightmap_raw.npy", raw)
        np.save(output_dir / "global_heightmap_filled.npy", filled)
        with (output_dir / "metadata.yaml").open("w", encoding="utf-8") as f:
            yaml.safe_dump(metadata, f, sort_keys=False)

    if args.skip_png:
        LOGGER.info("Skipping PNG previews")
    else:
        with timed_step("Write PNG previews"):
            save_png(output_dir / "global_heightmap_raw.png", raw, "global_heightmap_raw")
            save_png(output_dir / "global_heightmap_filled.png", filled, "global_heightmap_filled")
    LOGGER.info("Done. Output directory: %s", output_dir)


if __name__ == "__main__":
    main()
