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

#ifndef WUJI_HAND2_DRIVER__CONVERSION__JOINT_MAPPING_HPP_
#define WUJI_HAND2_DRIVER__CONVERSION__JOINT_MAPPING_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace wuji_hand2_driver::conversion {

enum class HandSide {
  kLeft,
  kRight,
};

class JointMapping {
 public:
  static constexpr std::size_t kJointCount = 20;

  explicit JointMapping(HandSide side) : side_(side), names_(make_names(side)) {}

  HandSide side() const noexcept {
    return side_;
  }

  std::optional<std::size_t> index_from_nid(std::uint8_t nid) const noexcept {
    for (std::size_t index = 0; index < kJointSpecs.size(); ++index) {
      if (kJointSpecs[index].nid == nid) {
        return index;
      }
    }
    return std::nullopt;
  }

  std::optional<std::uint8_t> nid_from_index(std::size_t index) const noexcept {
    if (index >= kJointSpecs.size()) {
      return std::nullopt;
    }
    return kJointSpecs[index].nid;
  }

  std::optional<std::string_view> name_from_index(std::size_t index) const noexcept {
    if (index >= names_.size()) {
      return std::nullopt;
    }
    return names_[index];
  }

  std::optional<std::string_view> name_from_nid(std::uint8_t nid) const noexcept {
    const auto index = index_from_nid(nid);
    if (!index) {
      return std::nullopt;
    }
    return names_[*index];
  }

  std::optional<std::size_t> index_from_name(std::string_view name) const noexcept {
    for (std::size_t index = 0; index < names_.size(); ++index) {
      if (names_[index] == name) {
        return index;
      }
    }
    return std::nullopt;
  }

 private:
  struct JointSpec {
    std::uint8_t nid;
    std::string_view name_body;
  };

  inline static constexpr std::array<JointSpec, kJointCount> kJointSpecs{{
      {1, "thumb_cmc_flex"},
      {2, "thumb_cmc_abd"},
      {3, "thumb_mcp"},
      {4, "thumb_ip"},
      {6, "index_finger_mcp_flex"},
      {7, "index_finger_mcp_abd"},
      {8, "index_finger_pip"},
      {9, "index_finger_dip"},
      {11, "middle_finger_mcp_flex"},
      {12, "middle_finger_mcp_abd"},
      {13, "middle_finger_pip"},
      {14, "middle_finger_dip"},
      {16, "ring_finger_mcp_flex"},
      {17, "ring_finger_mcp_abd"},
      {18, "ring_finger_pip"},
      {19, "ring_finger_dip"},
      {21, "pinky_mcp_flex"},
      {22, "pinky_mcp_abd"},
      {23, "pinky_pip"},
      {24, "pinky_dip"},
  }};

  static std::array<std::string, kJointCount> make_names(HandSide side) {
    std::array<std::string, kJointCount> names;
    const std::string_view prefix = side == HandSide::kLeft ? "l_" : "r_";
    for (std::size_t index = 0; index < names.size(); ++index) {
      names[index] = std::string{prefix} + std::string{kJointSpecs[index].name_body};
    }
    return names;
  }

  const HandSide side_;
  const std::array<std::string, kJointCount> names_;
};

}  // namespace wuji_hand2_driver::conversion

#endif  // WUJI_HAND2_DRIVER__CONVERSION__JOINT_MAPPING_HPP_
