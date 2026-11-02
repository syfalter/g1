#pragma once

#include "protocol.hpp"

#include <array>
#include <cstdint>
#include <string>

namespace rh56 {

constexpr std::size_t kJointCount = 6;
constexpr uint8_t kAllJointsMask = 0x3F;
constexpr uint8_t kMaximumSafeTemperatureC = 60;

enum class ResultCode : int32_t {
    kOk = 0,
    kInvalidArgument = 100,
    kHandUnavailable = 101,
    kBusy = 102,
    kPoseNotFound = 103,
    kSerialTimeout = 200,
    kWriteRejected = 201,
    kInvalidResponse = 202,
    kStorageError = 203,
    kJointFaulted = 300,
    kOverTemperature = 301,
    kFaultRemains = 302,
    kStaleCommand = 303,
};

struct Result {
    ResultCode code{ResultCode::kOk};
    std::string message{"ok"};
    uint8_t affected_mask{0};

    explicit operator bool() const { return code == ResultCode::kOk; }
};

struct HandState {
    bool online{false};
    Position feedback_q{};
    Position target_q{};
    RawValues force{};
    RawValues current{};
    RawValues force_limit{};
    RawValues current_limit{};
    std::array<bool, kJointCount> contact{};
    bool contact_monitoring{false};
    ByteValues error{};
    ByteValues status{};
    ByteValues temperature{};
    uint32_t lost_count{0};
    uint64_t last_command_id{0};
    uint64_t timestamp_ms{0};
};

inline JointMask DecodeMask(uint8_t bits)
{
    JointMask mask{};
    for (std::size_t i = 0; i < mask.size(); ++i)
        mask[i] = (bits & (uint8_t{1} << i)) != 0;
    return mask;
}

inline uint8_t EncodeMask(const JointMask& mask)
{
    uint8_t bits = 0;
    for (std::size_t i = 0; i < mask.size(); ++i)
        if (mask[i])
            bits |= uint8_t{1} << i;
    return bits;
}

}  // namespace rh56
