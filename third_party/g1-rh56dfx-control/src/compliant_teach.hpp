#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <optional>
#include <vector>

// Quasi-static, bidirectional admittance controller.
// FORCE_ACT is evaluated only while the actuator is stationary. After emitting
// one position step, all force samples are rejected until MarkMotionSettled().
class CompliantTeach
{
public:
    enum class Phase { Sense, WaitForSettle };

    struct Config
    {
        int close_delta{40};
        int open_delta{80};
        int open_floor{20};
        std::size_t filter_samples{3};
        std::size_t engage_samples{3};
        std::size_t release_samples{3};
        float step{0.005f};
        float maximum_travel{0.100f};
        int baseline_adapt_step{2};
        float baseline_slope_raw_per_q{0.0f};
        bool opposite_force_releases{false};
    };

    struct Update
    {
        std::optional<float> command;
        bool just_released{false};
        bool ignored_dynamic_force{false};
        bool at_limit{false};
        int filtered_force{0};
        int baseline{0};
        int close_threshold{0};
        int open_threshold{0};
        int direction{0};  // -1 close, +1 open, 0 neutral
        Phase phase{Phase::Sense};
    };

    CompliantTeach(int baseline, float initial_target)
        : CompliantTeach(baseline, initial_target, Config{})
    {
    }

    CompliantTeach(int baseline, float initial_target, const Config& config)
        : baseline_(baseline), initial_target_(initial_target),
          target_(initial_target), config_(config)
    {
        config_.filter_samples = std::max<std::size_t>(1, config_.filter_samples);
        config_.engage_samples = std::max<std::size_t>(1, config_.engage_samples);
        config_.release_samples = std::max<std::size_t>(1, config_.release_samples);
    }

    Update ObserveForce(int force_raw)
    {
        Update result = Snapshot();
        if (phase_ == Phase::WaitForSettle) {
            result.ignored_dynamic_force = true;
            return result;
        }

        force_window_.push_back(force_raw);
        while (force_window_.size() > config_.filter_samples)
            force_window_.pop_front();
        if (force_window_.size() < config_.filter_samples)
            return result;

        const int force = Median(force_window_);
        const int close_force_threshold = baseline_ - config_.close_delta;
        const int open_force_threshold =
            std::max(baseline_ + config_.open_delta, config_.open_floor);
        int candidate = 0;
        if (force < close_force_threshold)
            candidate = -1;
        else if (force > open_force_threshold)
            candidate = 1;

        if (config_.opposite_force_releases && last_direction_ != 0 &&
            candidate == -last_direction_) {
            engage_count_ = 0;
            candidate_direction_ = 0;
            if (++opposite_release_count_ >= config_.release_samples) {
                // At a new thumb pose, the passive mechanism can leave a large
                // opposite-signed no-load force. Treat the first reversal as a
                // release/re-zero event, never as an immediate reverse command.
                baseline_ = force;
                last_direction_ = 0;
                opposite_release_count_ = 0;
                release_count_ = 0;
                result.just_released = true;
            }
            result = MergeSnapshot(result);
            result.filtered_force = force;
            result.direction = 0;
            return result;
        }
        opposite_release_count_ = 0;

        if (candidate == 0) {
            candidate_direction_ = 0;
            engage_count_ = 0;
            baseline_ += std::clamp(force - baseline_,
                                    -config_.baseline_adapt_step,
                                    config_.baseline_adapt_step);
            if (last_direction_ != 0) {
                ++release_count_;
                if (release_count_ >= config_.release_samples) {
                    last_direction_ = 0;
                    release_count_ = 0;
                    result.just_released = true;
                }
            }
        } else {
            release_count_ = 0;
            if (candidate == candidate_direction_)
                ++engage_count_;
            else {
                candidate_direction_ = candidate;
                engage_count_ = 1;
            }

            if (engage_count_ >= config_.engage_samples) {
                const float minimum =
                    std::max(0.0f, initial_target_ - config_.maximum_travel);
                const float maximum =
                    std::min(1.0f, initial_target_ + config_.maximum_travel);
                const float next = std::clamp(
                    target_ + candidate * config_.step, minimum, maximum);
                engage_count_ = 0;
                candidate_direction_ = 0;
                last_direction_ = candidate;
                if (next == target_) {
                    result.at_limit = true;
                } else {
                    baseline_ += static_cast<int>(std::lround(
                        config_.baseline_slope_raw_per_q * (next - target_)));
                    target_ = next;
                    phase_ = Phase::WaitForSettle;
                    result.command = target_;
                }
            }
        }

        result = MergeSnapshot(result);
        result.filtered_force = force;
        result.direction = candidate;
        return result;
    }

    void MarkMotionSettled()
    {
        phase_ = Phase::Sense;
        force_window_.clear();
        candidate_direction_ = 0;
        engage_count_ = 0;
        opposite_release_count_ = 0;
    }

    float target() const { return target_; }
    int baseline() const { return baseline_; }
    int close_threshold() const { return baseline_ - config_.close_delta; }
    int open_threshold() const
    {
        return std::max(baseline_ + config_.open_delta, config_.open_floor);
    }
    float ActivationScore(int force_raw) const
    {
        if (force_raw < close_threshold())
            return static_cast<float>(close_threshold() - force_raw) /
                   config_.close_delta;
        if (force_raw > open_threshold())
            return static_cast<float>(force_raw - open_threshold()) /
                   config_.open_delta;
        return 0.0f;
    }
    Phase phase() const { return phase_; }

private:
    static int Median(const std::deque<int>& samples)
    {
        std::vector<int> sorted(samples.begin(), samples.end());
        std::sort(sorted.begin(), sorted.end());
        return sorted[sorted.size() / 2];
    }

    Update Snapshot() const
    {
        Update result;
        result.baseline = baseline_;
        result.close_threshold = baseline_ - config_.close_delta;
        result.open_threshold =
            std::max(baseline_ + config_.open_delta, config_.open_floor);
        result.phase = phase_;
        return result;
    }

    Update MergeSnapshot(Update result) const
    {
        result.baseline = baseline_;
        result.close_threshold = baseline_ - config_.close_delta;
        result.open_threshold =
            std::max(baseline_ + config_.open_delta, config_.open_floor);
        result.phase = phase_;
        return result;
    }

    int baseline_;
    float initial_target_;
    float target_;
    Config config_;
    Phase phase_{Phase::Sense};
    std::deque<int> force_window_;
    int candidate_direction_{0};
    int last_direction_{0};
    std::size_t engage_count_{0};
    std::size_t release_count_{0};
    std::size_t opposite_release_count_{0};
};
