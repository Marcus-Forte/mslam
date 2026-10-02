#include "slam/PointCloudIO.hh"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace mslam {
namespace {

enum class Encoding { Ascii, LittleEndian, BigEndian };
enum class ScalarType {
  Int8,
  UInt8,
  Int16,
  UInt16,
  Int32,
  UInt32,
  Float32,
  Float64
};

struct Property {
  std::string name;
  ScalarType type;
  bool is_list = false;
  ScalarType count_type = ScalarType::UInt8;
};

struct Element {
  std::string name;
  std::size_t count = 0;
  std::vector<Property> properties;
};

ScalarType parseScalarType(const std::string &name) {
  if (name == "char" || name == "int8") {
    return ScalarType::Int8;
  }
  if (name == "uchar" || name == "uint8") {
    return ScalarType::UInt8;
  }
  if (name == "short" || name == "int16") {
    return ScalarType::Int16;
  }
  if (name == "ushort" || name == "uint16") {
    return ScalarType::UInt16;
  }
  if (name == "int" || name == "int32") {
    return ScalarType::Int32;
  }
  if (name == "uint" || name == "uint32") {
    return ScalarType::UInt32;
  }
  if (name == "float" || name == "float32") {
    return ScalarType::Float32;
  }
  if (name == "double" || name == "float64") {
    return ScalarType::Float64;
  }
  throw std::runtime_error("Unsupported PLY property type: " + name);
}

std::size_t scalarSize(ScalarType type) {
  switch (type) {
  case ScalarType::Int8:
  case ScalarType::UInt8:
    return 1;
  case ScalarType::Int16:
  case ScalarType::UInt16:
    return 2;
  case ScalarType::Int32:
  case ScalarType::UInt32:
  case ScalarType::Float32:
    return 4;
  case ScalarType::Float64:
    return 8;
  }
  throw std::runtime_error("Invalid PLY property type");
}

double readBinaryScalar(std::istream &input, ScalarType type,
                        Encoding encoding) {
  std::array<unsigned char, 8> bytes{};
  const auto size = scalarSize(type);
  input.read(reinterpret_cast<char *>(bytes.data()),
             static_cast<std::streamsize>(size));
  if (!input) {
    throw std::runtime_error("Unexpected end of PLY data");
  }

  std::uint64_t bits = 0;
  for (std::size_t i = 0; i < size; ++i) {
    const auto byte_index =
        encoding == Encoding::LittleEndian ? i : size - i - 1;
    bits |= static_cast<std::uint64_t>(bytes[byte_index]) << (8 * i);
  }

  switch (type) {
  case ScalarType::Int8:
    return std::bit_cast<std::int8_t>(static_cast<std::uint8_t>(bits));
  case ScalarType::UInt8:
    return static_cast<std::uint8_t>(bits);
  case ScalarType::Int16:
    return std::bit_cast<std::int16_t>(static_cast<std::uint16_t>(bits));
  case ScalarType::UInt16:
    return static_cast<std::uint16_t>(bits);
  case ScalarType::Int32:
    return std::bit_cast<std::int32_t>(static_cast<std::uint32_t>(bits));
  case ScalarType::UInt32:
    return static_cast<std::uint32_t>(bits);
  case ScalarType::Float32:
    return std::bit_cast<float>(static_cast<std::uint32_t>(bits));
  case ScalarType::Float64:
    return std::bit_cast<double>(bits);
  }
  throw std::runtime_error("Invalid PLY property type");
}

double readAsciiScalar(std::istream &input) {
  std::string token;
  if (!(input >> token)) {
    throw std::runtime_error("Unexpected end of PLY data");
  }
  std::size_t parsed = 0;
  const double value = std::stod(token, &parsed);
  if (parsed != token.size()) {
    throw std::runtime_error("Invalid numeric value in PLY data: " + token);
  }
  return value;
}

double readScalar(std::istream &input, ScalarType type, Encoding encoding) {
  return encoding == Encoding::Ascii ? readAsciiScalar(input)
                                     : readBinaryScalar(input, type, encoding);
}

std::size_t readListCount(std::istream &input, ScalarType type,
                          Encoding encoding) {
  const double count = readScalar(input, type, encoding);
  if (!std::isfinite(count) || count < 0.0 ||
      count >= static_cast<double>(std::numeric_limits<std::size_t>::max()) ||
      count != static_cast<double>(static_cast<std::size_t>(count))) {
    throw std::runtime_error("Invalid list count in PLY data");
  }
  return static_cast<std::size_t>(count);
}

} // namespace

PointCloud readPlyPointCloud(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("Failed to open PLY file: " + path.string());
  }

  std::string line;
  if (!std::getline(input, line)) {
    throw std::runtime_error("Invalid PLY file header: " + path.string());
  }
  if (!line.empty() && line.back() == '\r') {
    line.pop_back();
  }
  if (line != "ply") {
    throw std::runtime_error("Invalid PLY file header: " + path.string());
  }

  Encoding encoding = Encoding::Ascii;
  bool has_encoding = false;
  bool has_vertex = false;
  bool has_end_header = false;
  std::vector<Element> elements;
  Element *current_element = nullptr;

  while (std::getline(input, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    std::istringstream header_line(line);
    std::string keyword;
    header_line >> keyword;
    if (keyword == "format") {
      std::string format;
      std::string version;
      header_line >> format >> version;
      if (version != "1.0") {
        throw std::runtime_error("Unsupported PLY format version: " + version);
      }
      if (format == "ascii") {
        encoding = Encoding::Ascii;
      } else if (format == "binary_little_endian") {
        encoding = Encoding::LittleEndian;
      } else if (format == "binary_big_endian") {
        encoding = Encoding::BigEndian;
      } else {
        throw std::runtime_error("Unsupported PLY format: " + format);
      }
      has_encoding = true;
    } else if (keyword == "element") {
      Element element;
      header_line >> element.name >> element.count;
      if (!header_line) {
        throw std::runtime_error("Invalid PLY element declaration");
      }
      has_vertex = has_vertex || element.name == "vertex";
      elements.push_back(std::move(element));
      current_element = &elements.back();
    } else if (keyword == "property") {
      if (current_element == nullptr) {
        throw std::runtime_error("PLY property declared without an element");
      }
      std::string type_or_list;
      header_line >> type_or_list;
      Property property;
      if (type_or_list == "list") {
        std::string count_type;
        std::string value_type;
        header_line >> count_type >> value_type >> property.name;
        property.is_list = true;
        property.count_type = parseScalarType(count_type);
        property.type = parseScalarType(value_type);
      } else {
        header_line >> property.name;
        property.type = parseScalarType(type_or_list);
      }
      if (!header_line) {
        throw std::runtime_error("Invalid PLY property declaration");
      }
      current_element->properties.push_back(std::move(property));
    } else if (keyword == "end_header") {
      has_end_header = true;
      break;
    }
  }

  if (!has_encoding || !has_vertex || !has_end_header) {
    throw std::runtime_error(
        "PLY header is missing format, vertex element, or end_header");
  }

  const auto vertex = std::find_if(
      elements.begin(), elements.end(),
      [](const Element &element) { return element.name == "vertex"; });
  bool has_x = false;
  bool has_y = false;
  bool has_z = false;
  for (const auto &property : vertex->properties) {
    has_x = has_x || property.name == "x";
    has_y = has_y || property.name == "y";
    has_z = has_z || property.name == "z";
  }
  if (!has_x || !has_y || !has_z) {
    throw std::runtime_error("PLY vertex element must have x, y, and z");
  }
  PointCloud cloud;
  cloud.reserve(vertex->count);

  for (const auto &element : elements) {
    for (std::size_t row = 0; row < element.count; ++row) {
      Point point{};
      for (const auto &property : element.properties) {
        if (property.is_list) {
          const auto count =
              readListCount(input, property.count_type, encoding);
          for (std::size_t i = 0; i < count; ++i) {
            (void)readScalar(input, property.type, encoding);
          }
          continue;
        }
        const double value = readScalar(input, property.type, encoding);
        if (element.name == "vertex") {
          if (property.name == "x") {
            point.x = static_cast<float>(value);
          } else if (property.name == "y") {
            point.y = static_cast<float>(value);
          } else if (property.name == "z") {
            point.z = static_cast<float>(value);
          } else if (property.name == "intensity") {
            point.intensity = static_cast<float>(value);
          }
        }
      }
      if (element.name == "vertex") {
        cloud.push_back(point);
      }
    }
  }
  return cloud;
}

} // namespace mslam
