// Copyright 2026 Wuji Technology
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "wuji_hand2_driver/conversion/tactile_decoder.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <iterator>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "wuji_hand2_driver/conversion/ros_header_conversion.hpp"

namespace wuji_hand2_driver::conversion {
namespace {

using Status = wuji_driver_common::stream::Status;
using Json = nlohmann::json;

constexpr std::size_t kSdkPayloadCapacity = 256;

enum class ScalarType {
  I8,
  U8,
  I16,
  U16,
  I32,
  U32,
  F32,
};

struct Field {
  ScalarType type;
  std::size_t offset;
  std::size_t width;
  double scale;
};

struct FieldRange {
  std::size_t begin;
  std::size_t end;
};

Status checked_size(const Json& object, const char* key, bool allow_zero, std::size_t& output) {
  const auto member = object.find(key);
  if (member == object.end() || member->is_boolean() ||
      (!member->is_number_integer() && !member->is_number_unsigned())) {
    return Status::failure(std::string(key) + " must be an integer");
  }

  std::uint64_t value = 0;
  if (member->is_number_unsigned()) {
    value = member->get<std::uint64_t>();
  } else {
    const std::int64_t signed_value = member->get<std::int64_t>();
    if (signed_value < 0) {
      return Status::failure(std::string(key) + " must be nonnegative");
    }
    value = static_cast<std::uint64_t>(signed_value);
  }
  if ((!allow_zero && value == 0) ||
      value > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max())) {
    return Status::failure(std::string(key) + " is outside the supported range");
  }
  output = static_cast<std::size_t>(value);
  return Status::success();
}

Status scalar_type(const std::string& name, ScalarType& type, std::size_t& width) {
  if (name == "i8") {
    type = ScalarType::I8;
    width = 1;
  } else if (name == "u8") {
    type = ScalarType::U8;
    width = 1;
  } else if (name == "i16") {
    type = ScalarType::I16;
    width = 2;
  } else if (name == "u16") {
    type = ScalarType::U16;
    width = 2;
  } else if (name == "i32") {
    type = ScalarType::I32;
    width = 4;
  } else if (name == "u32") {
    type = ScalarType::U32;
    width = 4;
  } else if (name == "f32") {
    type = ScalarType::F32;
    width = 4;
  } else {
    return Status::failure("unsupported fingertip scalar type: " + name);
  }
  return Status::success();
}

template <std::size_t N>
Status parse_fields(const Json& root, const char* key, std::size_t stride,
                    const std::array<std::string_view, N>& required_names,
                    std::array<Field, N>& required_fields) {
  const auto definitions = root.find(key);
  if (definitions == root.end() || !definitions->is_array()) {
    return Status::failure(std::string(key) + " must be an array");
  }

  std::array<bool, N> found{};
  std::unordered_set<std::string> names;
  std::vector<FieldRange> ranges;
  ranges.reserve(definitions->size());

  for (const auto& definition : *definitions) {
    if (!definition.is_object()) {
      return Status::failure(std::string(key) + " entries must be objects");
    }

    const auto name_member = definition.find("name");
    const auto type_member = definition.find("type");
    if (name_member == definition.end() || !name_member->is_string() ||
        type_member == definition.end() || !type_member->is_string()) {
      return Status::failure(std::string(key) + " entries require string name and type");
    }
    const std::string name = name_member->get<std::string>();
    if (name.empty() || !names.insert(name).second) {
      return Status::failure(std::string(key) + " contains an empty or duplicate name");
    }

    Field field{};
    Status status = scalar_type(type_member->get<std::string>(), field.type, field.width);
    if (!status) {
      return status;
    }
    status = checked_size(definition, "offset", true, field.offset);
    if (!status) {
      return status;
    }
    if (field.offset > stride || field.width > stride - field.offset) {
      return Status::failure(std::string(key) + " field exceeds its stride");
    }

    field.scale = 1.0;
    const auto scale_member = definition.find("scale");
    if (scale_member != definition.end()) {
      if (scale_member->is_boolean() || !scale_member->is_number()) {
        return Status::failure(std::string(key) + " field scale must be numeric");
      }
      field.scale = scale_member->get<double>();
      if (!std::isfinite(field.scale)) {
        return Status::failure(std::string(key) + " field scale must be finite");
      }
    }

    const FieldRange range{field.offset, field.offset + field.width};
    for (const auto& previous : ranges) {
      if (range.begin < previous.end && previous.begin < range.end) {
        return Status::failure(std::string(key) + " fields overlap");
      }
    }
    ranges.push_back(range);

    for (std::size_t index = 0; index < required_names.size(); ++index) {
      if (name == required_names[index]) {
        required_fields[index] = field;
        found[index] = true;
      }
    }
  }

  for (std::size_t index = 0; index < required_names.size(); ++index) {
    if (!found[index]) {
      return Status::failure(std::string(key) + " is missing semantic field " +
                             std::string(required_names[index]));
    }
  }
  return Status::success();
}

std::uint64_t read_unsigned_le(const std::uint8_t* data, std::size_t width) {
  std::uint64_t value = 0;
  for (std::size_t index = 0; index < width; ++index) {
    value |= static_cast<std::uint64_t>(data[index]) << (index * 8U);
  }
  return value;
}

double signed_value(std::uint64_t raw, std::size_t bits) {
  const std::uint64_t sign = std::uint64_t{1} << (bits - 1U);
  if ((raw & sign) == 0) {
    return static_cast<double>(raw);
  }
  const std::int64_t value =
      static_cast<std::int64_t>(raw) - static_cast<std::int64_t>(std::uint64_t{1} << bits);
  return static_cast<double>(value);
}

Status read_field(const Field& field, const std::uint8_t* base, double& output) {
  const std::uint8_t* const data = base + field.offset;
  double raw = 0.0;
  switch (field.type) {
    case ScalarType::I8:
      raw = signed_value(read_unsigned_le(data, 1), 8);
      break;
    case ScalarType::U8:
      raw = static_cast<double>(read_unsigned_le(data, 1));
      break;
    case ScalarType::I16:
      raw = signed_value(read_unsigned_le(data, 2), 16);
      break;
    case ScalarType::U16:
      raw = static_cast<double>(read_unsigned_le(data, 2));
      break;
    case ScalarType::I32:
      raw = signed_value(read_unsigned_le(data, 4), 32);
      break;
    case ScalarType::U32:
      raw = static_cast<double>(read_unsigned_le(data, 4));
      break;
    case ScalarType::F32: {
      const std::uint32_t bits = static_cast<std::uint32_t>(read_unsigned_le(data, 4));
      float value = 0.0F;
      static_assert(sizeof(value) == sizeof(bits), "f32 decoding requires a 32-bit float");
      std::memcpy(&value, &bits, sizeof(value));
      raw = value;
      break;
    }
  }

  const double scaled = raw * field.scale;
  if (!std::isfinite(raw) || !std::isfinite(scaled)) {
    return Status::failure("fingertip payload contains a nonfinite decoded value");
  }
  output = scaled;
  return Status::success();
}

}  // namespace

struct TactileDecoder::Layout {
  std::uint32_t digest;
  std::size_t point_count;
  std::size_t point_stride;
  std::size_t aggregate_stride;
  std::size_t expected_payload;
  std::array<Field, 3> point_fields;
  std::array<Field, 4> aggregate_fields;
};

TactileDecoder::Status TactileDecoder::configure(const WujiFingertipSensorInfo& info) {
  if (layout_) {
    return Status::failure("tactile decoder is already configured");
  }
  if (info.digest == 0) {
    return Status::failure("fingertip metadata digest must be nonzero");
  }
  if (!std::isfinite(info.rate_hz) || info.rate_hz <= 0.0F) {
    return Status::failure("fingertip metadata rate_hz must be finite and positive");
  }

  const auto terminator = std::find(std::begin(info.format), std::end(info.format), '\0');
  if (terminator == std::end(info.format)) {
    return Status::failure("fingertip metadata format is not NUL-terminated");
  }

  try {
    const Json root = Json::parse(info.format, terminator);
    if (!root.is_object()) {
      return Status::failure("fingertip metadata format must be a JSON object");
    }

    std::size_t version = 0;
    Status status = checked_size(root, "v", false, version);
    if (!status || version != 1) {
      return Status::failure("unsupported fingertip metadata version");
    }
    const auto encoding = root.find("encoding");
    if (encoding == root.end() || !encoding->is_string() ||
        encoding->get<std::string>() != "point_array") {
      return Status::failure("unsupported fingertip metadata encoding");
    }

    auto configured = std::make_shared<Layout>();
    configured->digest = info.digest;
    status = checked_size(root, "point_count", false, configured->point_count);
    if (!status) {
      return status;
    }
    status = checked_size(root, "point_stride", false, configured->point_stride);
    if (!status) {
      return status;
    }
    status = checked_size(root, "aggregate_stride", false, configured->aggregate_stride);
    if (!status) {
      return status;
    }

    if (configured->point_count >
        (std::numeric_limits<std::size_t>::max() - configured->aggregate_stride) /
            configured->point_stride) {
      return Status::failure("fingertip payload size overflows");
    }
    configured->expected_payload =
        configured->point_count * configured->point_stride + configured->aggregate_stride;
    if (configured->expected_payload > kSdkPayloadCapacity) {
      return Status::failure("fingertip payload exceeds the SDK capacity");
    }

    status =
        parse_fields(root, "point_fields", configured->point_stride,
                     std::array<std::string_view, 3>{"fx", "fy", "fz"}, configured->point_fields);
    if (!status) {
      return status;
    }
    status = parse_fields(root, "aggregate_fields", configured->aggregate_stride,
                          std::array<std::string_view, 4>{"fx", "fy", "fz", "temperature"},
                          configured->aggregate_fields);
    if (!status) {
      return status;
    }

    layout_ = std::move(configured);
    return Status::success();
  } catch (const std::exception& error) {
    return Status::failure(std::string("invalid fingertip metadata format: ") + error.what());
  } catch (...) {
    return Status::failure("invalid fingertip metadata format");
  }
}

TactileDecoder::Status TactileDecoder::decode(const WujiFingertipSensorData& input,
                                              wuji_msgs::msg::FingertipTactile& output) const {
  const std::shared_ptr<const Layout> layout = layout_;
  if (!layout) {
    return Status::failure("tactile decoder is not configured");
  }
  if (input.info_digest != layout->digest) {
    return Status::failure("fingertip metadata digest mismatch");
  }
  if (input.data_len != layout->expected_payload) {
    return Status::failure("fingertip payload length does not match metadata");
  }
  if (input.data_len != 0 && input.data == nullptr) {
    return Status::failure("fingertip payload pointer is null");
  }

  try {
    wuji_msgs::msg::FingertipTactile converted;
    Status status = to_ros_header(input.header, converted.header);
    if (!status) {
      return status;
    }
    converted.info_digest = input.info_digest;
    converted.point_forces.resize(layout->point_count);

    for (std::size_t index = 0; index < layout->point_count; ++index) {
      const std::uint8_t* const base = input.data + index * layout->point_stride;
      double values[3]{};
      for (std::size_t axis = 0; axis < 3; ++axis) {
        status = read_field(layout->point_fields[axis], base, values[axis]);
        if (!status) {
          return status;
        }
      }
      converted.point_forces[index].x = values[0];
      converted.point_forces[index].y = values[1];
      converted.point_forces[index].z = values[2];
    }

    const std::uint8_t* const aggregate = input.data + layout->point_count * layout->point_stride;
    double aggregate_values[4]{};
    for (std::size_t field = 0; field < 4; ++field) {
      status = read_field(layout->aggregate_fields[field], aggregate, aggregate_values[field]);
      if (!status) {
        return status;
      }
    }
    converted.total_force.x = aggregate_values[0];
    converted.total_force.y = aggregate_values[1];
    converted.total_force.z = aggregate_values[2];
    converted.temperature = static_cast<float>(aggregate_values[3]);
    if (!std::isfinite(converted.temperature)) {
      return Status::failure("fingertip payload temperature is outside the ROS field range");
    }

    output = std::move(converted);
    return Status::success();
  } catch (const std::exception& error) {
    return Status::failure(std::string("fingertip decode failed: ") + error.what());
  } catch (...) {
    return Status::failure("fingertip decode failed");
  }
}

bool TactileDecoder::configured() const noexcept {
  return static_cast<bool>(layout_);
}

std::uint32_t TactileDecoder::info_digest() const noexcept {
  return layout_ ? layout_->digest : 0;
}

bool TactileDecoder::digest_matches(const WujiFingertipSensorData& input) const noexcept {
  return layout_ && input.info_digest == layout_->digest;
}

}  // namespace wuji_hand2_driver::conversion
