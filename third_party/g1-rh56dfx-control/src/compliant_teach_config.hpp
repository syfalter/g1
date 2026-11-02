#pragma once

#include "compliant_teach.hpp"

#include <map>
#include <optional>
#include <string>
#include <yaml-cpp/yaml.h>

struct CompliantTeachJointOverride
{
    std::optional<int> close_delta;
    std::optional<int> open_delta;
    std::optional<int> open_floor;
    std::optional<float> baseline_slope_raw_per_q;
    std::optional<bool> opposite_force_releases;
};

struct CompliantTeachAppConfig
{
    std::string default_hand;
    std::string default_joint;
    std::string default_profile;
    int duration_seconds{30};
    bool full_range{false};
    std::map<std::string, float> profile_steps;
    CompliantTeach::Config controller;
    int motion_blank_ms{200};
    int extra_settle_ms{100};
    int loop_period_ms{25};
    int baseline_samples{15};
    int baseline_sample_period_ms{25};
    int safety_check_cycles{20};
    int maximum_current_mA{300};
    int maximum_temperature_C{60};
    bool thumb_arbitration_enabled{true};
    float thumb_dominance_ratio{2.0f};
    std::size_t thumb_engage_samples{2};
    std::map<std::string, CompliantTeachJointOverride> joint_overrides;
};

inline CompliantTeach::Config ApplyJointOverride(
    CompliantTeach::Config config, const CompliantTeachAppConfig& app,
    const std::string& hand, const std::string& joint)
{
    const auto it = app.joint_overrides.find(hand + "/" + joint);
    if (it == app.joint_overrides.end())
        return config;
    const auto& override = it->second;
    if (override.close_delta)
        config.close_delta = *override.close_delta;
    if (override.open_delta)
        config.open_delta = *override.open_delta;
    if (override.open_floor)
        config.open_floor = *override.open_floor;
    if (override.baseline_slope_raw_per_q)
        config.baseline_slope_raw_per_q =
            *override.baseline_slope_raw_per_q;
    if (override.opposite_force_releases)
        config.opposite_force_releases = *override.opposite_force_releases;
    return config;
}

inline CompliantTeachAppConfig LoadCompliantTeachConfig(
    const std::string& path)
{
    const YAML::Node root = YAML::LoadFile(path);
    CompliantTeachAppConfig config;

    config.default_hand = root["defaults"]["hand"].as<std::string>();
    config.default_joint = root["defaults"]["joint"].as<std::string>();
    config.default_profile = root["defaults"]["profile"].as<std::string>();
    config.duration_seconds =
        root["defaults"]["duration_seconds"].as<int>();
    config.full_range = root["defaults"]["full_range"].as<bool>();

    for (const auto& profile : root["profiles"])
        config.profile_steps.emplace(profile.first.as<std::string>(),
                                     profile.second.as<float>());

    const YAML::Node controller = root["controller"];
    config.controller.close_delta = controller["close_delta"].as<int>();
    config.controller.open_delta = controller["open_delta"].as<int>();
    config.controller.open_floor = controller["open_floor"].as<int>();
    config.controller.filter_samples =
        controller["filter_samples"].as<std::size_t>();
    config.controller.engage_samples =
        controller["engage_samples"].as<std::size_t>();
    config.controller.release_samples =
        controller["release_samples"].as<std::size_t>();
    config.controller.baseline_adapt_step =
        controller["baseline_adapt_step"].as<int>();
    config.controller.maximum_travel =
        controller["local_maximum_travel"].as<float>();

    const YAML::Node arbitration = root["thumb_arbitration"];
    config.thumb_arbitration_enabled = arbitration["enabled"].as<bool>();
    config.thumb_dominance_ratio =
        arbitration["dominance_ratio"].as<float>();
    config.thumb_engage_samples =
        arbitration["engage_samples"].as<std::size_t>();

    const YAML::Node overrides = root["joint_overrides"];
    for (const auto& hand_entry : overrides) {
        const std::string hand = hand_entry.first.as<std::string>();
        for (const auto& joint_entry : hand_entry.second) {
            const std::string joint = joint_entry.first.as<std::string>();
            const YAML::Node values = joint_entry.second;
            CompliantTeachJointOverride override;
            if (values["close_delta"])
                override.close_delta = values["close_delta"].as<int>();
            if (values["open_delta"])
                override.open_delta = values["open_delta"].as<int>();
            if (values["open_floor"])
                override.open_floor = values["open_floor"].as<int>();
            if (values["baseline_slope_raw_per_q"])
                override.baseline_slope_raw_per_q =
                    values["baseline_slope_raw_per_q"].as<float>();
            if (values["opposite_force_releases"])
                override.opposite_force_releases =
                    values["opposite_force_releases"].as<bool>();
            config.joint_overrides.emplace(hand + "/" + joint, override);
        }
    }

    const YAML::Node timing = root["timing"];
    config.motion_blank_ms = timing["motion_blank_ms"].as<int>();
    config.extra_settle_ms = timing["extra_settle_ms"].as<int>();
    config.loop_period_ms = timing["loop_period_ms"].as<int>();
    config.baseline_samples = timing["baseline_samples"].as<int>();
    config.baseline_sample_period_ms =
        timing["baseline_sample_period_ms"].as<int>();
    config.safety_check_cycles = timing["safety_check_cycles"].as<int>();

    const YAML::Node safety = root["safety"];
    config.maximum_current_mA = safety["maximum_current_mA"].as<int>();
    config.maximum_temperature_C =
        safety["maximum_temperature_C"].as<int>();
    return config;
}
