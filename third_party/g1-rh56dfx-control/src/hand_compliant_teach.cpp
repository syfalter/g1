#include "compliant_teach.hpp"
#include "compliant_teach_config.hpp"
#include "rh56/hand_client.hpp"

#include <unitree/robot/channel/channel_factory.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {
constexpr std::array<const char*, 6> kJointNames{
    "pinky", "ring", "middle", "index", "thumb-bend", "thumb-rotation"};
constexpr std::size_t kThumbBend = 4;
constexpr std::size_t kThumbRotation = 5;
volatile std::sig_atomic_t stop_requested = 0;

void StopHandler(int) { stop_requested = 1; }

std::filesystem::path ProjectRoot(const char* executable)
{
    return std::filesystem::canonical(executable)
        .parent_path().parent_path().parent_path();
}

int Median(std::vector<int16_t> values)
{
    std::sort(values.begin(), values.end());
    return values[values.size() / 2];
}

std::optional<std::size_t> JointIndex(const std::string& name)
{
    const auto it = std::find(kJointNames.begin(), kJointNames.end(), name);
    if (it == kJointNames.end())
        return std::nullopt;
    return static_cast<std::size_t>(it - kJointNames.begin());
}

std::optional<float> ProfileStep(const CompliantTeachAppConfig& config,
                                 const std::string& profile)
{
    const auto it = config.profile_steps.find(profile);
    if (it == config.profile_steps.end())
        return std::nullopt;
    return it->second;
}

struct HandRuntime
{
    HandRuntime(std::string hand_name, const rh56::JointMask& joint_selection,
                std::shared_ptr<rh56::HandClient> hand_client)
        : name(std::move(hand_name)), selected(joint_selection),
          client(std::move(hand_client))
    {}

    std::string name;
    rh56::JointMask selected{};
    std::shared_ptr<rh56::HandClient> client;
    rh56::StateReply state;
    std::array<std::unique_ptr<CompliantTeach>, 6> teach;
    rh56::JointMask pending{};
    std::chrono::steady_clock::time_point resume_at{};
    int sense_cycles{0};
    std::optional<std::size_t> thumb_owner;
    std::optional<std::size_t> thumb_candidate;
    std::size_t thumb_candidate_count{0};

    bool InDynamicBlank(std::chrono::steady_clock::time_point now) const
    {
        return std::any_of(pending.begin(), pending.end(),
                           [](bool value) { return value; }) &&
               now < resume_at;
    }
};

bool ReadState(HandRuntime& runtime)
{
    for (int attempt = 0; attempt < 3; ++attempt) {
        if (runtime.client->GetState(runtime.name, runtime.state, true) == 0 &&
            runtime.state.online && runtime.state.feedback_q.size() == 6 &&
            runtime.state.force.size() == 6 && runtime.state.current.size() == 6 &&
            runtime.state.error.size() == 6 && runtime.state.temperature.size() == 6)
            return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
}

bool WriteJoints(HandRuntime& runtime, const rh56::Position& position,
                 const rh56::JointMask& mask)
{
    rh56::SetTargetsRequest request;
    request.hand = runtime.name;
    request.joint_mask = rh56::EncodeMask(mask);
    request.q.assign(position.begin(), position.end());
    request.command_id = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
    request.timeout_ms = 5000;
    rh56::OperationReply reply;
    return runtime.client->SetTargets(request, reply) == 0;
}

bool SelectedTelemetrySafe(const HandRuntime& runtime,
                           const std::vector<int32_t>& current,
                           const std::vector<int32_t>& error,
                           const std::vector<int32_t>& temperature,
                           int maximum_current_mA, int maximum_temperature_C)
{
    for (std::size_t joint = 0; joint < runtime.selected.size(); ++joint) {
        if (!runtime.selected[joint])
            continue;
        if (std::abs(static_cast<int>(current[joint])) > maximum_current_mA ||
            error[joint] != 0 || temperature[joint] >= maximum_temperature_C)
            return false;
    }
    return true;
}

bool InitializeHand(HandRuntime& runtime,
                    const CompliantTeach::Config& config, bool full_range,
                    const CompliantTeachAppConfig& app_config)
{
    if (!ReadState(runtime)) {
        std::cerr << runtime.name << ": preflight telemetry read failed\n";
        return false;
    }

    for (std::size_t joint = 0; joint < runtime.selected.size(); ++joint) {
        if (runtime.selected[joint] &&
            (runtime.state.error[joint] != 0 ||
             runtime.state.temperature[joint] >=
                 app_config.maximum_temperature_C)) {
            std::cerr << runtime.name << '_' << kJointNames[joint]
                      << ": preflight rejected: fault or high temperature\n";
            return false;
        }
    }

    std::cout << runtime.name
              << ": keep selected joints unloaded while measuring baseline...\n";
    std::array<std::vector<int16_t>, 6> samples;
    for (int sample = 0; sample < app_config.baseline_samples; ++sample) {
        if (!ReadState(runtime)) {
            std::cerr << runtime.name << ": force baseline read failed\n";
            return false;
        }
        for (std::size_t joint = 0; joint < runtime.selected.size(); ++joint)
            if (runtime.selected[joint])
                samples[joint].push_back(
                    static_cast<int16_t>(runtime.state.force[joint]));
        std::this_thread::sleep_for(
            std::chrono::milliseconds(
                app_config.baseline_sample_period_ms));
    }

    for (std::size_t joint = 0; joint < runtime.selected.size(); ++joint) {
        if (!runtime.selected[joint])
            continue;
        const int baseline = Median(samples[joint]);
        const float initial = runtime.state.feedback_q[joint];
        CompliantTeach::Config joint_config = ApplyJointOverride(
            config, app_config, runtime.name, kJointNames[joint]);
        runtime.teach[joint] =
            std::make_unique<CompliantTeach>(baseline, initial, joint_config);
        const int open_threshold =
            std::max(baseline + joint_config.open_delta,
                     joint_config.open_floor);
        std::cout << std::fixed << std::setprecision(3)
                  << "  " << runtime.name << '_' << kJointNames[joint]
                  << " q=" << initial << " baseline=" << baseline
                  << " close_on<" << baseline - joint_config.close_delta
                  << " open_on>" << open_threshold
                  << " baseline_slope="
                  << joint_config.baseline_slope_raw_per_q
                  << " range="
                  << (full_range ? "[0.000,1.000]" : "initial+/-0.100")
                  << '\n';
    }
    return true;
}

bool ResumeAfterBlank(HandRuntime& runtime)
{
    if (!ReadState(runtime)) {
        std::cerr << runtime.name << ": post-step error read failed\n";
        return false;
    }
    for (std::size_t joint = 0; joint < runtime.pending.size(); ++joint) {
        if (!runtime.pending[joint])
            continue;
        if (runtime.state.error[joint] != 0) {
            std::cerr << runtime.name << '_' << kJointNames[joint]
                      << ": post-step error_bits=" << runtime.state.error[joint]
                      << '\n';
            return false;
        }
        runtime.teach[joint]->MarkMotionSettled();
        std::cout << "RESUME " << runtime.name << '_' << kJointNames[joint]
                  << " target=" << runtime.teach[joint]->target() << '\n';
    }
    runtime.pending.fill(false);
    return true;
}

bool SafetyCheck(const HandRuntime& runtime, int maximum_current_mA,
                 int maximum_temperature_C)
{
    if (!SelectedTelemetrySafe(runtime, runtime.state.current,
                               runtime.state.error, runtime.state.temperature,
                               maximum_current_mA,
                               maximum_temperature_C)) {
        std::cerr << runtime.name
                  << ": safety monitor stopped; holding last targets\n";
        return false;
    }
    return true;
}

bool ProcessHand(HandRuntime& runtime, int blank_ms, int settle_ms,
                 const CompliantTeachAppConfig& app_config)
{
    const auto now = std::chrono::steady_clock::now();
    if (runtime.InDynamicBlank(now))
        return true;
    if (std::any_of(runtime.pending.begin(), runtime.pending.end(),
                    [](bool value) { return value; })) {
        if (!ResumeAfterBlank(runtime))
            return false;
    }

    if (!ReadState(runtime)) {
        std::cerr << runtime.name << ": force read failed; holding targets\n";
        return false;
    }
    const auto& force = runtime.state.force;

    std::array<bool, 6> permitted{};
    permitted.fill(true);
    if (app_config.thumb_arbitration_enabled &&
        runtime.selected[kThumbBend] &&
        runtime.selected[kThumbRotation]) {
        if (runtime.thumb_owner) {
            permitted[kThumbBend] = *runtime.thumb_owner == kThumbBend;
            permitted[kThumbRotation] =
                *runtime.thumb_owner == kThumbRotation;
        } else {
            const float bend_score =
                runtime.teach[kThumbBend]->ActivationScore(force[kThumbBend]);
            const float rotation_score = runtime.teach[kThumbRotation]
                                             ->ActivationScore(
                                                 force[kThumbRotation]);
            std::optional<std::size_t> dominant;
            if (bend_score > 0.0f &&
                (rotation_score == 0.0f ||
                 bend_score >= rotation_score *
                                   app_config.thumb_dominance_ratio))
                dominant = kThumbBend;
            else if (rotation_score > 0.0f &&
                     (bend_score == 0.0f ||
                      rotation_score >= bend_score *
                                            app_config.thumb_dominance_ratio))
                dominant = kThumbRotation;

            if (dominant) {
                if (runtime.thumb_candidate == dominant)
                    ++runtime.thumb_candidate_count;
                else {
                    runtime.thumb_candidate = dominant;
                    runtime.thumb_candidate_count = 1;
                }
                if (runtime.thumb_candidate_count >=
                    app_config.thumb_engage_samples) {
                    runtime.thumb_owner = dominant;
                    runtime.thumb_candidate.reset();
                    runtime.thumb_candidate_count = 0;
                    std::cout << "THUMB_LOCK " << runtime.name << '_'
                              << kJointNames[*runtime.thumb_owner] << '\n';
                }
            } else {
                runtime.thumb_candidate.reset();
                runtime.thumb_candidate_count = 0;
            }

            if (runtime.thumb_owner) {
                permitted[kThumbBend] =
                    *runtime.thumb_owner == kThumbBend;
                permitted[kThumbRotation] =
                    *runtime.thumb_owner == kThumbRotation;
            } else if (bend_score > 0.0f || rotation_score > 0.0f) {
                // Do not let either independent controller act until one axis
                // has won the dominance/debounce decision.
                permitted[kThumbBend] = false;
                permitted[kThumbRotation] = false;
            }
        }
    }

    rh56::Position targets{};
    rh56::JointMask command_mask{};
    bool release_thumb_owner = false;
    for (std::size_t joint = 0; joint < runtime.selected.size(); ++joint) {
        if (!runtime.selected[joint] || !permitted[joint])
            continue;
        const auto update = runtime.teach[joint]->ObserveForce(force[joint]);
        if (update.command) {
            targets[joint] = *update.command;
            command_mask[joint] = true;
            runtime.pending[joint] = true;
            std::cout << (update.direction > 0 ? "OPEN " : "CLOSE ")
                      << runtime.name << '_' << kJointNames[joint]
                      << " force=" << update.filtered_force
                      << " baseline=" << update.baseline
                      << " target=" << *update.command << '\n';
        } else if (update.just_released) {
            std::cout << "HOLD " << runtime.name << '_' << kJointNames[joint]
                      << " target=" << runtime.teach[joint]->target()
                      << " baseline=" << update.baseline << '\n';
            if (runtime.thumb_owner && *runtime.thumb_owner == joint)
                release_thumb_owner = true;
        } else if (update.at_limit) {
            std::cout << "LIMIT " << runtime.name << '_' << kJointNames[joint]
                      << " target=" << runtime.teach[joint]->target() << '\n';
        }
    }

    if (release_thumb_owner) {
        std::cout << "THUMB_UNLOCK " << runtime.name << '\n';
        runtime.thumb_owner.reset();
        runtime.thumb_candidate.reset();
        runtime.thumb_candidate_count = 0;
    }

    if (std::any_of(command_mask.begin(), command_mask.end(),
                    [](bool value) { return value; })) {
        if (!WriteJoints(runtime, targets, command_mask)) {
            std::cerr << runtime.name
                      << ": position write failed; holding targets\n";
            return false;
        }
        runtime.resume_at = std::chrono::steady_clock::now() +
                            std::chrono::milliseconds(blank_ms + settle_ms);
        std::cout << "BLANK " << runtime.name << " all force channels for "
                  << blank_ms + settle_ms << " ms\n";
    }

    if (++runtime.sense_cycles % app_config.safety_check_cycles == 0 &&
        !SafetyCheck(runtime, app_config.maximum_current_mA,
                     app_config.maximum_temperature_C))
        return false;
    return true;
}

void PrintUsage(const char* executable)
{
    std::cerr
        << "Usage: " << executable
        << " [--config path] [--hand left|right|both]"
           " [--joint pinky|ring|middle|index|thumb-bend|thumb-rotation|all]"
           " [--profile slow|medium|fast] [--step q]"
           " [--duration seconds] [--close-delta raw]"
           " [--open-delta raw] [--open-floor raw]"
           " [--blank-ms ms] [--settle-ms ms]"
           " [--full-range] [--network INTERFACE] [--execute]\n";
}
}  // namespace

int main(int argc, char** argv)
{
    std::string config_path =
        (ProjectRoot(argv[0]) / "config" / "compliant_teach.yaml").string();
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--config" && i + 1 < argc)
            config_path = argv[++i];
    }

    CompliantTeachAppConfig app_config;
    try {
        app_config = LoadCompliantTeachConfig(config_path);
    } catch (const std::exception& error) {
        std::cerr << "Cannot load compliant-teach config " << config_path
                  << ": " << error.what() << '\n';
        return 2;
    }

    bool execute = false;
    bool full_range = app_config.full_range;
    bool custom_step = false;
    std::string hand_selection = app_config.default_hand;
    std::string joint_selection = app_config.default_joint;
    std::string profile = app_config.default_profile;
    std::string network;
    int duration_seconds = app_config.duration_seconds;
    int blank_ms = app_config.motion_blank_ms;
    int settle_ms = app_config.extra_settle_ms;
    CompliantTeach::Config teach_config = app_config.controller;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--execute")
            execute = true;
        else if (arg == "--config" && i + 1 < argc)
            ++i;
        else if (arg == "--full-range")
            full_range = true;
        else if (arg == "--hand" && i + 1 < argc)
            hand_selection = argv[++i];
        else if (arg == "--joint" && i + 1 < argc)
            joint_selection = argv[++i];
        else if (arg == "--profile" && i + 1 < argc)
            profile = argv[++i];
        else if (arg == "--network" && i + 1 < argc)
            network = argv[++i];
        else if (arg == "--step" && i + 1 < argc) {
            teach_config.step = std::stof(argv[++i]);
            custom_step = true;
        } else if (arg == "--duration" && i + 1 < argc)
            duration_seconds = std::stoi(argv[++i]);
        else if (arg == "--close-delta" && i + 1 < argc)
            teach_config.close_delta = std::stoi(argv[++i]);
        else if (arg == "--open-delta" && i + 1 < argc)
            teach_config.open_delta = std::stoi(argv[++i]);
        else if (arg == "--open-floor" && i + 1 < argc)
            teach_config.open_floor = std::stoi(argv[++i]);
        else if (arg == "--blank-ms" && i + 1 < argc)
            blank_ms = std::stoi(argv[++i]);
        else if (arg == "--settle-ms" && i + 1 < argc)
            settle_ms = std::stoi(argv[++i]);
        else {
            PrintUsage(argv[0]);
            return 2;
        }
    }

    const auto profile_step = ProfileStep(app_config, profile);
    if (!custom_step && profile_step)
        teach_config.step = *profile_step;
    const bool valid_hand = hand_selection == "left" ||
                            hand_selection == "right" ||
                            hand_selection == "both";
    const bool valid_joint = joint_selection == "all" ||
                             JointIndex(joint_selection).has_value();
    if (!valid_hand || !valid_joint || !profile_step ||
        duration_seconds <= 0 || duration_seconds > 300 ||
        !std::isfinite(teach_config.step) || teach_config.step < 0.001f ||
        teach_config.step > 0.020f || teach_config.close_delta < 10 ||
        teach_config.open_delta < 10 || blank_ms < 20 || blank_ms > 1000 ||
        settle_ms < 20 || settle_ms > 1000 ||
        app_config.loop_period_ms < 5 ||
        app_config.baseline_samples < 1 ||
        app_config.baseline_sample_period_ms < 1 ||
        app_config.safety_check_cycles < 1 ||
        app_config.maximum_current_mA < 1 ||
        app_config.maximum_temperature_C < 1 ||
        app_config.thumb_dominance_ratio < 1.0f ||
        app_config.thumb_engage_samples < 1) {
        PrintUsage(argv[0]);
        return 2;
    }
    if (full_range)
        teach_config.maximum_travel = 1.0f;

    rh56::JointMask selected{};
    if (joint_selection == "all")
        selected.fill(true);
    else
        selected[*JointIndex(joint_selection)] = true;

    std::signal(SIGINT, StopHandler);
    std::signal(SIGTERM, StopHandler);

    try {
        unitree::robot::ChannelFactory::Instance()->Init(0, network);
        auto client = std::make_shared<rh56::HandClient>();
        client->SetTimeout(2.0f);
        client->Init();
        std::vector<HandRuntime> hands;
        if (hand_selection == "left" || hand_selection == "both")
            hands.push_back(HandRuntime{"left", selected, client});
        if (hand_selection == "right" || hand_selection == "both")
            hands.push_back(HandRuntime{"right", selected, client});

        std::cout << "config=" << config_path << " profile=" << profile
                  << " step=" << teach_config.step
                  << " hand=" << hand_selection
                  << " joint=" << joint_selection << '\n';
        for (auto& hand : hands)
            if (!InitializeHand(hand, teach_config, full_range, app_config))
                return 1;

        if (!execute) {
            std::cout << "DRY RUN: no motion command sent\n";
            return 0;
        }

        std::cout << "ACTIVE bidirectional quasi-static teaching. "
                     "Ctrl+C stops it.\n";
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::seconds(duration_seconds);
        while (!stop_requested && std::chrono::steady_clock::now() < deadline) {
            for (auto& hand : hands)
                if (!ProcessHand(hand, blank_ms, settle_ms, app_config))
                    return 1;
            std::this_thread::sleep_for(
                std::chrono::milliseconds(app_config.loop_period_ms));
        }

        std::cout << "DONE: final targets\n";
        for (const auto& hand : hands)
            for (std::size_t joint = 0; joint < hand.selected.size(); ++joint)
                if (hand.selected[joint])
                    std::cout << "  " << hand.name << '_' << kJointNames[joint]
                              << '=' << hand.teach[joint]->target() << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
