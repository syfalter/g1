#pragma once

#include "hand_types.hpp"
#include "transport.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <thread>

namespace rh56 {

class HandController
{
public:
    explicit HandController(std::unique_ptr<Transport> transport)
        : transport_(std::move(transport))
    {}

    Result SetTargets(const Position& position, uint8_t mask_bits,
                      uint64_t command_id = 0)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!ValidMask(mask_bits))
            return Invalid("joint_mask must select bits 0..5");

        const JointMask mask = DecodeMask(mask_bits);
        for (std::size_t i = 0; i < position.size(); ++i) {
            if (mask[i] && (!std::isfinite(position[i]) || position[i] < 0.0f ||
                            position[i] > 1.0f))
                return Invalid("selected q values must be in [0,1]");
        }

        if (!health_valid_ && !RefreshHealthLocked())
            return CommunicationFailure("cannot read safety telemetry");
        for (std::size_t i = 0; i < mask.size(); ++i) {
            if (mask[i] && (state_.error[i] != 0 || state_.status[i] == 7))
                return {ResultCode::kJointFaulted,
                        "selected joint is faulted",
                        static_cast<uint8_t>(uint8_t{1} << i)};
        }

        if (!transport_->WritePosition(position, mask))
            return {ResultCode::kWriteRejected,
                    "position command was not acknowledged", 0};
        ResetContactLocked();
        for (std::size_t i = 0; i < mask.size(); ++i)
            if (mask[i])
                state_.target_q[i] = position[i];
        target_initialized_ = true;
        state_.last_command_id = command_id;
        StampOnline();
        return {ResultCode::kOk, "ok", mask_bits};
    }

    Result HoldCurrent()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        Position current{};
        if (!transport_->ReadPosition(current))
            return CommunicationFailure("cannot read current position");
        JointMask all{};
        all.fill(true);
        if (!transport_->WritePosition(current, all))
            return {ResultCode::kWriteRejected,
                    "current-position hold was not acknowledged", 0};
        ResetContactLocked();
        state_.feedback_q = current;
        state_.target_q = current;
        target_initialized_ = true;
        StampOnline();
        return {ResultCode::kOk, "current position held", kAllJointsMask};
    }

    Result ApplyGrip(const Position& position, int16_t force_grams,
                     int16_t current_ma, uint64_t command_id = 0)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (force_grams < 50 || force_grams > 1000 ||
            current_ma < 50 || current_ma > 300)
            return Invalid("grip force or current is outside the safe range");
        for (const float q : position)
            if (!std::isfinite(q) || q < 0.0f || q > 1.0f)
                return Invalid("grip q values must be in [0,1]");

        if (!transport_->ReadWords(0x063A, state_.current) ||
            !RefreshHealthLocked())
            return CommunicationFailure("pre-grip safety telemetry read failed");
        for (std::size_t i = 0; i < state_.current.size(); ++i) {
            if (state_.error[i] != 0 || state_.status[i] == 7)
                return {ResultCode::kJointFaulted,
                        "joint is faulted", static_cast<uint8_t>(1u << i)};
            if (state_.temperature[i] >= kMaximumSafeTemperatureC)
                return {ResultCode::kOverTemperature,
                        "joint is too hot", static_cast<uint8_t>(1u << i)};
            if (std::abs(static_cast<int>(state_.current[i])) >= 300)
                return {ResultCode::kWriteRejected,
                        "joint current is already at the safety limit",
                        static_cast<uint8_t>(1u << i)};
        }

        RawValues force_limits{};
        RawValues current_limits{};
        force_limits.fill(force_grams);
        current_limits.fill(current_ma);
        JointMask all{};
        all.fill(true);
        if (!transport_->WriteWords(0x03FC, current_limits, 1500) ||
            !transport_->WriteWords(0x05DA, force_limits, 1000) ||
            !transport_->WritePosition(position, all))
            return {ResultCode::kWriteRejected,
                    "grip configuration or target was not acknowledged", 0};
        ResetContactLocked();

        state_.current_limit = current_limits;
        state_.force_limit = force_limits;
        state_.target_q = position;
        state_.last_command_id = command_id;
        target_initialized_ = true;
        StampOnline();
        return {ResultCode::kOk, "ok", kAllJointsMask};
    }

    Result RefreshPosition()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!transport_->ReadPosition(state_.feedback_q))
            return CommunicationFailure("position read failed");
        InitializeTargetLocked();
        StampOnline();
        return {};
    }

    Result RefreshState()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!transport_->ReadPosition(state_.feedback_q) ||
            !transport_->ReadWords(0x062E, state_.force) ||
            !transport_->ReadWords(0x063A, state_.current) ||
            !transport_->ReadWords(0x05DA, state_.force_limit) ||
            !transport_->ReadWords(0x03FC, state_.current_limit) ||
            !RefreshHealthLocked())
            return CommunicationFailure("telemetry read failed");
        InitializeTargetLocked();
        StampOnline();
        return {};
    }

    Result RefreshForce(bool monitor_contact, int16_t threshold)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!transport_->ReadWords(0x062E, state_.force)) {
            ResetContactLocked();
            return CommunicationFailure("force read failed");
        }
        UpdateContactLocked(monitor_contact, threshold);
        StampOnline();
        return {};
    }

    Result ClearFault(uint8_t requested_mask)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!ValidMask(requested_mask))
            return Invalid("joint_mask must select bits 0..5");
        if (!transport_->ReadPosition(state_.feedback_q) ||
            !RefreshHealthLocked())
            return CommunicationFailure("pre-clear telemetry read failed");
        InitializeTargetLocked();

        uint8_t faulted_mask = 0;
        JointMask recover{};
        for (std::size_t i = 0; i < recover.size(); ++i) {
            const bool faulted = state_.error[i] != 0 || state_.status[i] == 7;
            recover[i] = faulted;
            if (faulted && (requested_mask & (uint8_t{1} << i)))
                faulted_mask |= uint8_t{1} << i;
            if (state_.error[i] != 0 &&
                state_.temperature[i] >= kMaximumSafeTemperatureC)
                return {ResultCode::kOverTemperature,
                        "overtemperature faults must cool and auto-clear",
                        static_cast<uint8_t>(uint8_t{1} << i)};
        }
        if (faulted_mask == 0)
            return {ResultCode::kOk, "no requested fault is present", 0};

        if (!transport_->WritePosition(state_.feedback_q, recover))
            return {ResultCode::kWriteRejected,
                    "failed to hold faulted joints", 0};

        bool has_error_bits = false;
        for (std::size_t i = 0; i < state_.error.size(); ++i)
            has_error_bits |= (requested_mask & (uint8_t{1} << i)) != 0 &&
                              state_.error[i] != 0;
        if (has_error_bits && !transport_->ClearErrors())
            return {ResultCode::kWriteRejected,
                    "clear-error write was not acknowledged", 0};
        if (!transport_->WritePosition(state_.feedback_q, recover))
            return {ResultCode::kWriteRejected,
                    "failed to restore current-position hold", 0};

        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        if (!RefreshHealthLocked())
            return CommunicationFailure("post-clear telemetry read failed");

        uint8_t remaining = 0;
        for (std::size_t i = 0; i < state_.error.size(); ++i)
            if ((requested_mask & (uint8_t{1} << i)) != 0 &&
                (state_.error[i] != 0 || state_.status[i] == 7))
                remaining |= uint8_t{1} << i;
        StampOnline();
        if (remaining != 0)
            return {ResultCode::kFaultRemains,
                    "fault remains after clear", remaining};
        return {ResultCode::kOk, "fault cleared; no travel commanded",
                faulted_mask};
    }

    HandState GetState() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return state_;
    }

private:
    static bool ValidMask(uint8_t mask)
    {
        return mask != 0 && (mask & ~kAllJointsMask) == 0;
    }

    static Result Invalid(const std::string& message)
    {
        return {ResultCode::kInvalidArgument, message, 0};
    }

    bool RefreshHealthLocked()
    {
        health_valid_ = transport_->ReadBytes(0x0646, state_.error) &&
                        transport_->ReadBytes(0x064C, state_.status) &&
                        transport_->ReadBytes(0x0652, state_.temperature);
        return health_valid_;
    }

    Result CommunicationFailure(const std::string& message)
    {
        state_.online = false;
        ++state_.lost_count;
        health_valid_ = false;
        return {ResultCode::kSerialTimeout, message, 0};
    }

    void StampOnline()
    {
        state_.online = true;
        state_.timestamp_ms = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
    }

    void InitializeTargetLocked()
    {
        if (!target_initialized_) {
            state_.target_q = state_.feedback_q;
            target_initialized_ = true;
        }
    }

    void UpdateContactLocked(bool enabled, int16_t threshold)
    {
        if (!enabled || threshold <= 0) {
            ResetContactLocked();
            return;
        }
        if (contact_samples_ < 3) {
            for (std::size_t i = 0; i < kJointCount; ++i)
                contact_baseline_[i] +=
                    (state_.force[i] - contact_baseline_[i]) /
                    static_cast<float>(contact_samples_ + 1);
            state_.contact.fill(false);
            state_.contact_monitoring = ++contact_samples_ == 3;
            return;
        }
        state_.contact_monitoring = true;
        const float release = std::max(1.0f, threshold * 0.5f);
        for (std::size_t i = 0; i < kJointCount; ++i) {
            const float delta = std::abs(state_.force[i] - contact_baseline_[i]);
            state_.contact[i] = delta >=
                (state_.contact[i] ? release : static_cast<float>(threshold));
            if (!state_.contact[i])
                contact_baseline_[i] = contact_baseline_[i] * 0.95f +
                                       state_.force[i] * 0.05f;
        }
    }

    void ResetContactLocked()
    {
        state_.contact.fill(false);
        state_.contact_monitoring = false;
        contact_samples_ = 0;
    }

    std::unique_ptr<Transport> transport_;
    mutable std::mutex mutex_;
    HandState state_{};
    std::array<float, kJointCount> contact_baseline_{};
    uint8_t contact_samples_{0};
    bool health_valid_{false};
    bool target_initialized_{false};
};

}  // namespace rh56
