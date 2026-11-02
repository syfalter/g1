#pragma once

#include <array>
#include <cstdint>

namespace g1_kdl_ik
{

constexpr std::size_t kDirectionBins = 42;
constexpr std::size_t kRollBins = 8;
constexpr std::size_t kOrientationBins = kDirectionBins * kRollBins;
constexpr std::size_t kOrientationBytes = (kOrientationBins + 7) / 8;
constexpr std::uint8_t kBoundaryFlag = 1;

#pragma pack(push, 1)
struct CapabilityRecordV1
{
  std::int32_t ix;
  std::int32_t iy;
  std::int32_t iz;
  std::uint32_t count;
  std::uint16_t orientation_count;
  std::uint8_t flags;
  std::uint8_t reserved;
  std::uint8_t orientation_mask[kOrientationBytes];
  float seed[7];
};
#pragma pack(pop)

static_assert(sizeof(CapabilityRecordV1) == 90,
              "Unexpected capability-map v1 record size");

}  // namespace g1_kdl_ik
