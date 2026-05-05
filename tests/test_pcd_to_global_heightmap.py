import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

from pcd_to_global_heightmap import fill_nans_local_mean, optional_float, points_to_heightmap


class PcdToGlobalHeightmapTest(unittest.TestCase):
    def test_optional_float_can_disable_filter(self):
        self.assertEqual(optional_float("1.0"), 1.0)
        self.assertIsNone(optional_float("none"))
        self.assertIsNone(optional_float("OFF"))

    def test_points_to_heightmap_z_modes(self):
        points = np.array(
            [
                [0.1, 0.1, 2.0],
                [0.2, 0.1, 1.0],
                [1.1, 0.1, 5.0],
                [0.1, 1.1, 4.0],
            ],
            dtype=np.float64,
        )

        raw_min, metadata = points_to_heightmap(points, 1.0, "min")
        raw_max, _ = points_to_heightmap(points, 1.0, "max")
        raw_mean, _ = points_to_heightmap(points, 1.0, "mean")

        self.assertEqual(metadata["width"], 2)
        self.assertEqual(metadata["height"], 2)
        np.testing.assert_allclose(raw_min, [[1.0, 5.0], [4.0, np.nan]], equal_nan=True)
        np.testing.assert_allclose(raw_max, [[2.0, 5.0], [4.0, np.nan]], equal_nan=True)
        np.testing.assert_allclose(raw_mean, [[1.5, 5.0], [4.0, np.nan]], equal_nan=True)

    def test_fill_nans_local_mean_uses_neighbor_means(self):
        try:
            import scipy  # noqa: F401
        except ImportError:
            self.skipTest("scipy is not installed")

        raw = np.array(
            [
                [1.0, np.nan, 3.0],
                [np.nan, np.nan, np.nan],
                [7.0, np.nan, 9.0],
            ],
            dtype=np.float32,
        )

        filled = fill_nans_local_mean(raw, max_iterations=1, filter_size=3)

        np.testing.assert_allclose(
            filled,
            [
                [1.0, 2.0, 3.0],
                [4.0, 5.0, 6.0],
                [7.0, 8.0, 9.0],
            ],
        )


if __name__ == "__main__":
    unittest.main()
