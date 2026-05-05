#include "height_measurements/npy_reader.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>

namespace height_measurements {
namespace {

uint16_t read_u16_le(std::ifstream &in) {
  unsigned char b[2] = {0, 0};
  in.read(reinterpret_cast<char *>(b), 2);
  if (!in) {
    throw std::runtime_error("failed to read npy uint16 header length");
  }
  return static_cast<uint16_t>(b[0]) | (static_cast<uint16_t>(b[1]) << 8);
}

uint32_t read_u32_le(std::ifstream &in) {
  unsigned char b[4] = {0, 0, 0, 0};
  in.read(reinterpret_cast<char *>(b), 4);
  if (!in) {
    throw std::runtime_error("failed to read npy uint32 header length");
  }
  return static_cast<uint32_t>(b[0]) | (static_cast<uint32_t>(b[1]) << 8) |
         (static_cast<uint32_t>(b[2]) << 16) |
         (static_cast<uint32_t>(b[3]) << 24);
}

std::string regex_capture(const std::string &text, const std::regex &pattern,
                          const std::string &name) {
  std::smatch match;
  if (!std::regex_search(text, match, pattern) || match.size() < 2) {
    throw std::runtime_error("npy header missing field: " + name);
  }
  return match[1].str();
}

std::vector<std::size_t> parse_shape(const std::string &shape_text) {
  std::vector<std::size_t> dims;
  std::stringstream ss(shape_text);
  std::string item;
  while (std::getline(ss, item, ',')) {
    item.erase(std::remove_if(item.begin(), item.end(), ::isspace), item.end());
    if (item.empty()) {
      continue;
    }
    dims.push_back(static_cast<std::size_t>(std::stoull(item)));
  }
  return dims;
}

bool host_is_little_endian() {
  const uint16_t value = 1;
  return *reinterpret_cast<const unsigned char *>(&value) == 1;
}

} // namespace

NpyArray2D load_npy_2d_float(const std::string &path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    throw std::runtime_error("failed to open npy file: " + path);
  }

  unsigned char magic[6] = {};
  in.read(reinterpret_cast<char *>(magic), 6);
  const unsigned char expected_magic[6] = {0x93, 'N', 'U', 'M', 'P', 'Y'};
  if (!in || std::memcmp(magic, expected_magic, 6) != 0) {
    throw std::runtime_error("not a NumPy .npy file: " + path);
  }

  unsigned char version[2] = {};
  in.read(reinterpret_cast<char *>(version), 2);
  if (!in) {
    throw std::runtime_error("failed to read npy version: " + path);
  }

  std::size_t header_len = 0;
  if (version[0] == 1) {
    header_len = read_u16_le(in);
  } else if (version[0] == 2 || version[0] == 3) {
    header_len = read_u32_le(in);
  } else {
    throw std::runtime_error("unsupported npy major version in: " + path);
  }

  std::string header(header_len, '\0');
  in.read(header.data(), static_cast<std::streamsize>(header_len));
  if (!in) {
    throw std::runtime_error("failed to read npy header: " + path);
  }

  const std::regex descr_pattern(R"(['"]descr['"]\s*:\s*['"]([^'"]+)['"])");
  const std::regex fortran_pattern(R"(['"]fortran_order['"]\s*:\s*(True|False))");
  const std::regex shape_pattern(R"(['"]shape['"]\s*:\s*\(([^\)]*)\))");

  const std::string descr = regex_capture(header, descr_pattern, "descr");
  const std::string fortran_order =
      regex_capture(header, fortran_pattern, "fortran_order");
  const std::string shape_text = regex_capture(header, shape_pattern, "shape");
  const std::vector<std::size_t> shape = parse_shape(shape_text);

  if (fortran_order != "False") {
    throw std::runtime_error("Fortran-order npy arrays are not supported: " +
                             path);
  }
  if (shape.size() != 2) {
    throw std::runtime_error("heightmap npy must be 2D: " + path);
  }
  if (shape[0] == 0 || shape[1] == 0) {
    throw std::runtime_error("heightmap npy has an empty dimension: " + path);
  }

  const bool little_endian_descr =
      descr.size() >= 3 && (descr[0] == '<' || descr[0] == '|' || descr[0] == '=');
  if (!little_endian_descr || (!host_is_little_endian() && descr[0] != '|')) {
    throw std::runtime_error("unsupported npy endian/descr: " + descr);
  }

  const std::size_t rows = shape[0];
  const std::size_t cols = shape[1];
  const std::size_t count = rows * cols;
  NpyArray2D array;
  array.rows = rows;
  array.cols = cols;
  array.dtype = descr;
  array.data.resize(count);

  if (descr == "<f4" || descr == "|f4" || descr == "=f4") {
    in.read(reinterpret_cast<char *>(array.data.data()),
            static_cast<std::streamsize>(count * sizeof(float)));
    if (!in) {
      throw std::runtime_error("failed to read float32 npy payload: " + path);
    }
  } else if (descr == "<f8" || descr == "|f8" || descr == "=f8") {
    std::vector<double> tmp(count);
    in.read(reinterpret_cast<char *>(tmp.data()),
            static_cast<std::streamsize>(count * sizeof(double)));
    if (!in) {
      throw std::runtime_error("failed to read float64 npy payload: " + path);
    }
    for (std::size_t i = 0; i < count; ++i) {
      array.data[i] = static_cast<float>(tmp[i]);
    }
  } else {
    throw std::runtime_error("unsupported npy dtype " + descr +
                             "; expected float32 or float64");
  }

  return array;
}

} // namespace height_measurements
