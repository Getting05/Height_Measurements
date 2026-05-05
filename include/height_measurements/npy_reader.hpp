#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace height_measurements {

struct NpyArray2D {
  std::size_t rows = 0;
  std::size_t cols = 0;
  std::string dtype;
  std::vector<float> data;
};

/// Load an uncompressed C-order 2D NumPy .npy array with float32 or float64 data.
NpyArray2D load_npy_2d_float(const std::string &path);

} // namespace height_measurements
