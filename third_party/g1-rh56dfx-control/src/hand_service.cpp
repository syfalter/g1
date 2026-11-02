#include "rh56/hand_api.hpp"
#include "rh56/hand_controller.hpp"
#include "rh56/persistent_store.hpp"

#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/server/server.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>

namespace {

constexpr const char* kRightDevice =
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTABQDTD-if01-port0";
constexpr const char* kLeftDevice =
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTABQDTD-if02-port0";
constexpr auto kLoopPeriod = std::chrono::milliseconds(20);
constexpr uint32_t kDefaultCommandTimeoutMs = 300;

std::atomic_bool running{true};

struct Config {
    std::string hand{"both"};
    std::string network;
    std::filesystem::path poses_file;
    std::filesystem::path settings_file;
    bool execute{false};
};

void HandleSignal(int) { running = false; }

std::filesystem::path ProjectRoot(const char* executable)
{
    return std::filesystem::weakly_canonical(executable)
        .parent_path().parent_path().parent_path();
}

class PidFile
{
public:
    explicit PidFile(const std::filesystem::path& root)
        : path_(root / "run" / "hand_service.pid")
    {
        std::filesystem::create_directories(path_.parent_path());
        std::ifstream input(path_);
        int owner = 0;
        if ((input >> owner) && owner > 0 &&
            std::filesystem::exists("/proc/" + std::to_string(owner)))
            throw std::runtime_error("Another hand controller is active (PID " +
                                     std::to_string(owner) + ")");
        std::ofstream(path_, std::ios::trunc) << getpid() << '\n';
    }

    ~PidFile()
    {
        std::ifstream input(path_);
        int owner = 0;
        if ((input >> owner) && owner == getpid()) {
            std::error_code error;
            std::filesystem::remove(path_, error);
        }
    }

private:
    std::filesystem::path path_;
};

Config ParseArgs(int argc, char** argv)
{
    Config config;
    config.poses_file = ProjectRoot(argv[0]) / "data" / "hand_poses.json";
    config.settings_file = ProjectRoot(argv[0]) / "data" / "hand_settings.json";
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if (argument == "--execute") {
            config.execute = true;
        } else if ((argument == "--hand" || argument == "--network" ||
                    argument == "--poses-file") &&
                   i + 1 < argc) {
            const std::string value = argv[++i];
            if (argument == "--hand")
                config.hand = value;
            else if (argument == "--poses-file")
                config.poses_file = value;
            else
                config.network = value;
        } else {
            throw std::invalid_argument(
                "Usage: hand_service --execute [--hand left|right|both] "
                "[--network INTERFACE] [--poses-file PATH]");
        }
    }
    if (!config.execute || (config.hand != "left" && config.hand != "right" &&
                            config.hand != "both"))
        throw std::invalid_argument(
            "Usage: hand_service --execute [--hand left|right|both] "
            "[--network INTERFACE] [--poses-file PATH]");
    return config;
}

template <typename T>
std::vector<int32_t> Integers(const T& values)
{
    std::vector<int32_t> output(values.size());
    std::transform(values.begin(), values.end(), output.begin(),
                   [](const auto value) { return static_cast<int32_t>(value); });
    return output;
}

class HandRuntime
{
public:
    explicit HandRuntime(const Config& config)
        : poses_(config.poses_file), settings_(config.settings_file)
    {
        if (config.hand == "right" || config.hand == "both")
            right_ = MakeController(kRightDevice);
        if (config.hand == "left" || config.hand == "both")
            left_ = MakeController(kLeftDevice);

        state_publisher_ = std::make_shared<StatePublisher>("rt/inspire/state");
        state_publisher_->InitChannel();
        command_subscriber_ =
            std::make_shared<CommandSubscriber>("rt/inspire/cmd");
        command_subscriber_->InitChannel(
            [this](const void* message) { OnDdsCommand(message); }, 1);
    }

    rh56::OperationReply SetTargets(const rh56::SetTargetsRequest& request)
    {
        rh56::OperationReply reply;
        reply.request_id = request.command_id;
        std::unique_lock<std::mutex> operation(control_mutex_, std::try_to_lock);
        if (!operation)
            return Fail(reply, rh56::ResultCode::kBusy,
                        "another RPC control operation is active");
        auto* controller = ControllerFor(request.hand);
        if (!controller)
            return Fail(reply, rh56::ResultCode::kHandUnavailable,
                        "requested hand is not enabled");
        if (request.q.size() != rh56::kJointCount || request.joint_mask == 0 ||
            request.joint_mask > rh56::kAllJointsMask ||
            request.timeout_ms < 50 || request.timeout_ms > 5000)
            return Fail(reply, rh56::ResultCode::kInvalidArgument,
                        "expected six q values, mask 1..63 and timeout 50..5000 ms");
        if (!TryArmRpcWatchdog(request.hand, request.timeout_ms))
            return Fail(reply, rh56::ResultCode::kBusy,
                        "DDS position stream currently owns this hand");

        rh56::Position position{};
        std::copy(request.q.begin(), request.q.end(), position.begin());
        const auto result = controller->SetTargets(
            position, static_cast<uint8_t>(request.joint_mask),
            request.command_id);
        Fill(reply, result, controller->GetState());
        if (!result)
            CancelRpcWatchdog(request.hand);
        return reply;
    }

    rh56::OperationReply ClearFault(const rh56::ClearFaultRequest& request)
    {
        rh56::OperationReply reply;
        reply.request_id = request.request_id;
        std::unique_lock<std::mutex> operation(control_mutex_, std::try_to_lock);
        if (!operation)
            return Fail(reply, rh56::ResultCode::kBusy,
                        "another RPC control operation is active");
        auto* controller = ControllerFor(request.hand);
        if (!controller)
            return Fail(reply, rh56::ResultCode::kHandUnavailable,
                        "requested hand is not enabled");
        if (request.joint_mask == 0 || request.joint_mask > rh56::kAllJointsMask)
            return Fail(reply, rh56::ResultCode::kInvalidArgument,
                        "joint_mask must be in 1..63");

        BeginMaintenance(request.hand);
        const auto result = controller->ClearFault(
            static_cast<uint8_t>(request.joint_mask));
        Fill(reply, result, controller->GetState());
        return reply;
    }

    rh56::OperationReply ApplyGrip(const rh56::GripRequest& request)
    {
        rh56::OperationReply reply;
        reply.request_id = request.command_id;
        std::unique_lock<std::mutex> operation(control_mutex_, std::try_to_lock);
        if (!operation)
            return Fail(reply, rh56::ResultCode::kBusy,
                        "another RPC control operation is active");
        auto* controller = ControllerFor(request.hand);
        if (!controller)
            return Fail(reply, rh56::ResultCode::kHandUnavailable,
                        "requested hand is not enabled");
        if (request.q.size() != rh56::kJointCount ||
            request.force_grams < 50 || request.force_grams > 1000 ||
            request.current_ma < 50 || request.current_ma > 300 ||
            request.timeout_ms < 50 || request.timeout_ms > 5000)
            return Fail(reply, rh56::ResultCode::kInvalidArgument,
                        "invalid grip position, force, current or timeout");
        if (!TryArmRpcWatchdog(request.hand, request.timeout_ms))
            return Fail(reply, rh56::ResultCode::kBusy,
                        "DDS position stream currently owns this hand");

        rh56::Position position{};
        std::copy(request.q.begin(), request.q.end(), position.begin());
        const auto result = controller->ApplyGrip(
            position, static_cast<int16_t>(request.force_grams),
            static_cast<int16_t>(request.current_ma), request.command_id);
        Fill(reply, result, controller->GetState());
        if (!result)
            CancelRpcWatchdog(request.hand);
        return reply;
    }

    rh56::StateReply GetState(const std::string& hand, bool refresh)
    {
        rh56::StateReply reply;
        reply.hand = hand;
        auto* controller = ControllerFor(hand);
        if (!controller) {
            reply.code = static_cast<int32_t>(rh56::ResultCode::kHandUnavailable);
            reply.message = "requested hand is not enabled";
            return reply;
        }
        const auto refresh_result = refresh ? controller->RefreshState()
                                            : rh56::Result{};
        const auto state = controller->GetState();
        reply.code = static_cast<int32_t>(refresh_result.code);
        reply.message = refresh_result.message;
        reply.online = state.online;
        reply.feedback_q.assign(state.feedback_q.begin(), state.feedback_q.end());
        reply.target_q.assign(state.target_q.begin(), state.target_q.end());
        reply.force = Integers(state.force);
        reply.current = Integers(state.current);
        reply.force_limit = Integers(state.force_limit);
        reply.current_limit = Integers(state.current_limit);
        reply.contact.assign(state.contact.begin(), state.contact.end());
        reply.contact_monitoring = state.contact_monitoring;
        reply.error = Integers(state.error);
        reply.status = Integers(state.status);
        reply.temperature = Integers(state.temperature);
        reply.lost_count = state.lost_count;
        reply.last_command_id = state.last_command_id;
        reply.timestamp_ms = state.timestamp_ms;
        return reply;
    }

    rh56::OperationReply Hold(const rh56::HoldRequest& request)
    {
        rh56::OperationReply reply;
        reply.request_id = request.request_id;
        auto* controller = ControllerFor(request.hand);
        if (!controller)
            return Fail(reply, rh56::ResultCode::kHandUnavailable,
                        "requested hand is not enabled");
        pose_cancel_requested_ = true;
        std::lock_guard<std::mutex> operation(control_mutex_);
        BeginMaintenance(request.hand);
        const auto result = controller->HoldCurrent();
        Fill(reply, result, controller->GetState());
        return reply;
    }

    rh56::PoseReply Poses(const rh56::PoseRequest& request)
    {
        rh56::PoseReply reply;
        reply.request_id = request.request_id;
        try {
            if (request.action == "list") {
                reply.poses = poses_.List();
            } else if (request.action == "save") {
                reply.pose = poses_.Save(request);
            } else if (request.action == "rename") {
                const auto pose = poses_.Rename(request.id, request.name);
                if (!pose)
                    return Fail(reply, rh56::ResultCode::kPoseNotFound,
                                "pose not found");
                reply.pose = *pose;
            } else if (request.action == "set_delays") {
                const auto pose = poses_.SetDelays(request.id, request.delays_ms);
                if (!pose)
                    return Fail(reply, rh56::ResultCode::kPoseNotFound,
                                "pose not found");
                reply.pose = *pose;
            } else if (request.action == "delete") {
                if (!poses_.Delete(request.id))
                    return Fail(reply, rh56::ResultCode::kPoseNotFound,
                                "pose not found");
            } else if (request.action == "execute") {
                return ExecutePose(request);
            } else {
                return Fail(reply, rh56::ResultCode::kInvalidArgument,
                            "unknown pose action");
            }
            reply.message = "ok";
            return reply;
        } catch (const std::invalid_argument& error) {
            return Fail(reply, rh56::ResultCode::kInvalidArgument, error.what());
        } catch (const std::exception& error) {
            return Fail(reply, rh56::ResultCode::kStorageError, error.what());
        }
    }

    rh56::SettingsMessage Settings(const rh56::SettingsMessage& request)
    {
        rh56::SettingsMessage reply;
        reply.request_id = request.request_id;
        try {
            if (request.write) {
                const auto& value = request.settings;
                if (!rh56::ValidSettings(value))
                    return Fail(reply, rh56::ResultCode::kInvalidArgument,
                                "invalid persistent hand settings");
                settings_.Set(value);
            }
            reply.settings = settings_.Get();
            reply.message = "ok";
            return reply;
        } catch (const std::exception& error) {
            return Fail(reply, rh56::ResultCode::kStorageError, error.what());
        }
    }

    rh56::PoseReply ExecutePose(const rh56::PoseRequest& request)
    {
        rh56::PoseReply reply;
        reply.request_id = request.request_id;
        if (request.hand != "left" && request.hand != "right" &&
            request.hand != "both")
            return Fail(reply, rh56::ResultCode::kInvalidArgument,
                        "hand must be left, right or both");
        if (request.timeout_ms < 50 || request.timeout_ms > 5000)
            return Fail(reply, rh56::ResultCode::kInvalidArgument,
                        "timeout_ms must be in 50..5000");

        const auto pose = poses_.Find(request.id);
        if (!pose)
            return Fail(reply, rh56::ResultCode::kPoseNotFound,
                        "pose not found");
        reply.pose = *pose;

        std::vector<std::string> hands;
        if (request.hand != "left" && right_)
            hands.emplace_back("right");
        if (request.hand != "right" && left_)
            hands.emplace_back("left");
        if (hands.empty())
            return Fail(reply, rh56::ResultCode::kHandUnavailable,
                        "requested hand is not enabled");

        std::unique_lock<std::mutex> operation(control_mutex_, std::try_to_lock);
        if (!operation)
            return Fail(reply, rh56::ResultCode::kBusy,
                        "another RPC control operation is active");
        pose_cancel_requested_ = false;

        const uint32_t maximum_delay = *std::max_element(
            pose->delays_ms.begin(), pose->delays_ms.end());
        if (!TryArmRpcWatchdogs(hands, maximum_delay + request.timeout_ms + 500))
            return Fail(reply, rh56::ResultCode::kBusy,
                        "DDS position stream currently owns a requested hand");

        std::vector<std::string> controlled;
        auto stop = [&](rh56::ResultCode code, const std::string& message) {
            for (const auto& hand : controlled)
                ControllerFor(hand)->HoldCurrent();
            CancelRpcWatchdogs(hands);
            return Fail(reply, code, message);
        };

        for (const auto& hand : hands) {
            auto* controller = ControllerFor(hand);
            const auto refresh = controller->RefreshPosition();
            if (!refresh)
                return stop(refresh.code, hand + ": " + refresh.message);
            const auto current = controller->GetState().feedback_q;
            const auto hold = controller->SetTargets(
                current, rh56::kAllJointsMask, request.request_id);
            if (!hold)
                return stop(hold.code, hand + ": " + hold.message);
            controlled.push_back(hand);
        }

        std::map<uint32_t, uint8_t> groups;
        for (std::size_t joint = 0; joint < rh56::kJointCount; ++joint)
            groups[pose->delays_ms[joint]] |= uint8_t{1} << joint;

        const auto started = std::chrono::steady_clock::now();
        for (const auto& [delay_ms, mask] : groups) {
            if (!WaitForPoseDelay(started +
                                  std::chrono::milliseconds(delay_ms)))
                return stop(rh56::ResultCode::kStaleCommand,
                            running ? "pose execution was cancelled"
                                    : "service is stopping");
            for (const auto& hand : hands) {
                rh56::Position target{};
                const auto& values = hand == "right" ? pose->right : pose->left;
                std::copy(values.begin(), values.end(), target.begin());
                const auto result = ControllerFor(hand)->SetTargets(
                    target, mask, request.request_id);
                if (!result)
                    return stop(result.code, hand + ": " + result.message);
            }
        }

        ArmRpcWatchdogs(hands, request.timeout_ms);
        reply.message = "ok";
        reply.duration_ms = maximum_delay;
        reply.affected_hands = hands;
        return reply;
    }

    void HoldEnabled()
    {
        if (right_)
            LogFailure("right shutdown hold", right_->HoldCurrent());
        if (left_)
            LogFailure("left shutdown hold", left_->HoldCurrent());
    }

    void Step(uint64_t iteration)
    {
        ApplyDdsCommands();
        ApplyWatchdogs();
        Refresh(right_.get(), 0, iteration);
        Refresh(left_.get(), 1, iteration);
        PublishState();
    }

private:
    using CommandMessage = unitree_go::msg::dds_::MotorCmds_;
    using StateMessage = unitree_go::msg::dds_::MotorStates_;
    using CommandSubscriber = unitree::robot::ChannelSubscriber<CommandMessage>;
    using StatePublisher = unitree::robot::ChannelPublisher<StateMessage>;

    struct DdsCommand {
        std::array<float, 12> q{};
        std::chrono::steady_clock::time_point received_at{};
        uint64_t sequence{0};
        uint64_t applied_sequence{0};
        bool received{false};
        std::array<bool, 2> timed_out{false, false};
    };

    struct RpcWatchdog {
        std::chrono::steady_clock::time_point deadline{};
        bool active{false};
    };

    static std::unique_ptr<rh56::HandController> MakeController(
        const std::string& device)
    {
        return std::make_unique<rh56::HandController>(
            std::make_unique<rh56::SerialTransport>(device));
    }

    rh56::HandController* ControllerFor(const std::string& hand)
    {
        if (hand == "right")
            return right_.get();
        if (hand == "left")
            return left_.get();
        return nullptr;
    }

    void OnDdsCommand(const void* message)
    {
        const auto& command = *static_cast<const CommandMessage*>(message);
        if (command.cmds().size() < 12)
            return;
        std::array<float, 12> q{};
        for (std::size_t i = 0; i < q.size(); ++i) {
            q[i] = command.cmds()[i].q();
            if (!std::isfinite(q[i]) || q[i] < 0.0f || q[i] > 1.0f)
                return;
        }
        std::lock_guard<std::mutex> lock(command_mutex_);
        dds_.q = q;
        dds_.received_at = std::chrono::steady_clock::now();
        dds_.received = true;
        dds_.timed_out = {false, false};
        ++dds_.sequence;
    }

    bool TryArmRpcWatchdog(const std::string& hand, uint32_t timeout_ms)
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - dds_.received_at).count();
        const std::size_t index = hand == "right" ? 0 : 1;
        if (dds_.received && age <= kDefaultCommandTimeoutMs &&
            !dds_.timed_out[index])
            return false;
        rpc_watchdogs_[index].deadline = std::chrono::steady_clock::now() +
                                         std::chrono::milliseconds(timeout_ms);
        rpc_watchdogs_[index].active = true;
        return true;
    }

    bool TryArmRpcWatchdogs(const std::vector<std::string>& hands,
                            uint32_t timeout_ms)
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        const auto now = std::chrono::steady_clock::now();
        const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - dds_.received_at).count();
        for (const auto& hand : hands) {
            const std::size_t index = hand == "right" ? 0 : 1;
            if (dds_.received && age <= kDefaultCommandTimeoutMs &&
                !dds_.timed_out[index])
                return false;
        }
        for (const auto& hand : hands) {
            auto& watchdog = rpc_watchdogs_[hand == "right" ? 0 : 1];
            watchdog.deadline = now + std::chrono::milliseconds(timeout_ms);
            watchdog.active = true;
        }
        return true;
    }

    void ArmRpcWatchdogs(const std::vector<std::string>& hands,
                         uint32_t timeout_ms)
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(timeout_ms);
        for (const auto& hand : hands) {
            auto& watchdog = rpc_watchdogs_[hand == "right" ? 0 : 1];
            watchdog.deadline = deadline;
            watchdog.active = true;
        }
    }

    void CancelRpcWatchdogs(const std::vector<std::string>& hands)
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        for (const auto& hand : hands)
            rpc_watchdogs_[hand == "right" ? 0 : 1].active = false;
    }

    bool WaitForPoseDelay(std::chrono::steady_clock::time_point deadline) const
    {
        while (running && !pose_cancel_requested_) {
            const auto now = std::chrono::steady_clock::now();
            if (now >= deadline)
                return true;
            std::this_thread::sleep_until(
                std::min(deadline, now + std::chrono::milliseconds(20)));
        }
        return false;
    }

    bool RpcActive(std::size_t index) const
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        return rpc_watchdogs_[index].active &&
               std::chrono::steady_clock::now() < rpc_watchdogs_[index].deadline;
    }

    void BeginMaintenance(const std::string& hand)
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        const std::size_t index = hand == "right" ? 0 : 1;
        dds_.timed_out[index] = true;
        rpc_watchdogs_[index].deadline = std::chrono::steady_clock::now() +
                                         std::chrono::milliseconds(500);
        rpc_watchdogs_[index].active = true;
    }

    void CancelRpcWatchdog(const std::string& hand)
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        rpc_watchdogs_[hand == "right" ? 0 : 1].active = false;
    }

    void ApplyDdsCommands()
    {
        std::array<float, 12> q{};
        uint64_t sequence = 0;
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            if (!dds_.received || dds_.sequence == dds_.applied_sequence)
                return;
            const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - dds_.received_at).count();
            if (age > kDefaultCommandTimeoutMs) {
                dds_.applied_sequence = dds_.sequence;
                return;
            }
            q = dds_.q;
            sequence = dds_.sequence;
            dds_.applied_sequence = sequence;
        }

        if (right_ && !RpcActive(0)) {
            rh56::Position position{};
            std::copy_n(q.begin(), 6, position.begin());
            LogFailure("right DDS command",
                       right_->SetTargets(position, rh56::kAllJointsMask,
                                          sequence));
        }
        if (left_ && !RpcActive(1)) {
            rh56::Position position{};
            std::copy_n(q.begin() + 6, 6, position.begin());
            LogFailure("left DDS command",
                       left_->SetTargets(position, rh56::kAllJointsMask,
                                         sequence));
        }
    }

    void ApplyWatchdogs()
    {
        const auto now = std::chrono::steady_clock::now();
        std::array<bool, 2> hold{false, false};
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            if (dds_.received &&
                now - dds_.received_at >
                    std::chrono::milliseconds(kDefaultCommandTimeoutMs)) {
                for (std::size_t i = 0; i < hold.size(); ++i) {
                    if (!dds_.timed_out[i]) {
                        dds_.timed_out[i] = true;
                        hold[i] = true;
                    }
                }
            }
            for (std::size_t i = 0; i < hold.size(); ++i) {
                if (rpc_watchdogs_[i].active && now >= rpc_watchdogs_[i].deadline) {
                    rpc_watchdogs_[i].active = false;
                    hold[i] = true;
                }
            }
        }
        if (hold[0] && right_)
            LogFailure("right command watchdog", right_->HoldCurrent());
        if (hold[1] && left_)
            LogFailure("left command watchdog", left_->HoldCurrent());
    }

    bool HandIdle(std::size_t index) const
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        return (!dds_.received || dds_.timed_out[index]) &&
               !rpc_watchdogs_[index].active;
    }

    void Refresh(rh56::HandController* controller, std::size_t index,
                 uint64_t iteration)
    {
        if (!controller)
            return;
        const auto result = iteration % 25 == 0
                                ? controller->RefreshState()
                                : controller->RefreshPosition();
        LogFailure("state refresh", result);
        if (iteration % 5 == 0)
            LogFailure("force refresh", controller->RefreshForce(
                HandIdle(index),
                static_cast<int16_t>(settings_.Get().contact_threshold)));
    }

    void PublishState()
    {
        StateMessage message;
        message.states().resize(12);
        FillMotorStates(message, right_.get(), 0);
        FillMotorStates(message, left_.get(), 6);
        state_publisher_->Write(message);
    }

    static void FillMotorStates(StateMessage& message,
                                const rh56::HandController* controller,
                                std::size_t offset)
    {
        if (!controller) {
            for (std::size_t i = 0; i < 6; ++i)
                message.states()[offset + i].lost(1);
            return;
        }
        const auto state = controller->GetState();
        for (std::size_t i = 0; i < 6; ++i) {
            auto& motor = message.states()[offset + i];
            motor.q(state.feedback_q[i]);
            motor.mode(state.status[i]);
            motor.temperature(state.temperature[i]);
            motor.lost(state.lost_count);
            motor.reserve()[0] = state.error[i];
            motor.reserve()[1] = state.status[i];
        }
    }

    static void Fill(rh56::OperationReply& reply, const rh56::Result& result,
                     const rh56::HandState& state)
    {
        reply.code = static_cast<int32_t>(result.code);
        reply.message = result.message;
        reply.affected_mask = result.affected_mask;
        reply.error = Integers(state.error);
        reply.status = Integers(state.status);
        reply.temperature = Integers(state.temperature);
    }

    template <typename Reply>
    static Reply Fail(Reply reply, rh56::ResultCode code,
                      const std::string& message)
    {
        reply.code = static_cast<int32_t>(code);
        reply.message = message;
        return reply;
    }

    static void LogFailure(const char* operation, const rh56::Result& result)
    {
        if (!result)
            std::cerr << operation << ": " << result.message << '\n';
    }

    std::unique_ptr<rh56::HandController> right_;
    std::unique_ptr<rh56::HandController> left_;
    rh56::PoseStore poses_;
    rh56::SettingsStore settings_;
    std::shared_ptr<CommandSubscriber> command_subscriber_;
    std::shared_ptr<StatePublisher> state_publisher_;
    std::mutex control_mutex_;
    std::atomic_bool pose_cancel_requested_{false};
    mutable std::mutex command_mutex_;
    DdsCommand dds_{};
    std::array<RpcWatchdog, 2> rpc_watchdogs_{};
};

class HandRpcServer final : public unitree::robot::Server
{
public:
    explicit HandRpcServer(HandRuntime& runtime)
        : Server(rh56::kServiceName), runtime_(runtime)
    {}

    void Init() override
    {
        SetApiVersion(rh56::kApiVersion);
        UT_ROBOT_SERVER_REG_API_HANDLER_NO_LEASE(
            rh56::kApiSetTargets, &HandRpcServer::HandleSetTargets);
        UT_ROBOT_SERVER_REG_API_HANDLER_NO_LEASE(
            rh56::kApiClearFault, &HandRpcServer::HandleClearFault);
        UT_ROBOT_SERVER_REG_API_HANDLER_NO_LEASE(
            rh56::kApiGetState, &HandRpcServer::HandleGetState);
        UT_ROBOT_SERVER_REG_API_HANDLER_NO_LEASE(
            rh56::kApiApplyGrip, &HandRpcServer::HandleApplyGrip);
        UT_ROBOT_SERVER_REG_API_HANDLER_NO_LEASE(
            rh56::kApiHold, &HandRpcServer::HandleHold);
        UT_ROBOT_SERVER_REG_API_HANDLER_NO_LEASE(
            rh56::kApiPoses, &HandRpcServer::HandlePoses);
        UT_ROBOT_SERVER_REG_API_HANDLER_NO_LEASE(
            rh56::kApiSettings, &HandRpcServer::HandleSettings);
    }

private:
    int32_t HandleSetTargets(const std::string& parameter, std::string& data)
    {
        return Handle<rh56::SetTargetsRequest>(
            parameter, data,
            [this](const auto& request) { return runtime_.SetTargets(request); });
    }

    int32_t HandleClearFault(const std::string& parameter, std::string& data)
    {
        return Handle<rh56::ClearFaultRequest>(
            parameter, data,
            [this](const auto& request) { return runtime_.ClearFault(request); });
    }

    int32_t HandleGetState(const std::string& parameter, std::string& data)
    {
        return Handle<rh56::GetStateRequest>(
            parameter, data,
            [this](const auto& request) {
                return runtime_.GetState(request.hand, request.refresh);
            });
    }

    int32_t HandleApplyGrip(const std::string& parameter, std::string& data)
    {
        return Handle<rh56::GripRequest>(
            parameter, data,
            [this](const auto& request) { return runtime_.ApplyGrip(request); });
    }

    int32_t HandleHold(const std::string& parameter, std::string& data)
    {
        return Handle<rh56::HoldRequest>(
            parameter, data,
            [this](const auto& request) { return runtime_.Hold(request); });
    }

    int32_t HandlePoses(const std::string& parameter, std::string& data)
    {
        return Handle<rh56::PoseRequest>(
            parameter, data,
            [this](const auto& request) { return runtime_.Poses(request); });
    }

    int32_t HandleSettings(const std::string& parameter, std::string& data)
    {
        return Handle<rh56::SettingsMessage>(
            parameter, data,
            [this](const auto& request) { return runtime_.Settings(request); });
    }

    template <typename Request, typename Handler>
    static int32_t Handle(const std::string& parameter, std::string& data,
                          Handler handler)
    {
        try {
            Request request;
            unitree::common::FromJsonString(parameter, request);
            data = unitree::common::ToJsonString(handler(request));
        } catch (const std::exception& error) {
            rh56::OperationReply reply;
            reply.code = static_cast<int32_t>(
                rh56::ResultCode::kInvalidArgument);
            reply.message = std::string("invalid request: ") + error.what();
            data = unitree::common::ToJsonString(reply);
        }
        return 0;
    }

    HandRuntime& runtime_;
};

}  // namespace

int main(int argc, char** argv)
{
    try {
        const Config config = ParseArgs(argc, argv);
        PidFile pid_file(ProjectRoot(argv[0]));
        std::signal(SIGINT, HandleSignal);
        std::signal(SIGTERM, HandleSignal);

        unitree::robot::ChannelFactory::Instance()->Init(0, config.network);
        HandRuntime runtime(config);
        HandRpcServer rpc(runtime);
        rpc.Init();
        rpc.Start();

        std::cout << "RH56 hand service ready: hand=" << config.hand
                  << " network=" << (config.network.empty() ? "default" : config.network)
                  << '\n';
        uint64_t iteration = 0;
        while (running) {
            const auto next = std::chrono::steady_clock::now() + kLoopPeriod;
            runtime.Step(iteration++);
            std::this_thread::sleep_until(next);
        }
        runtime.HoldEnabled();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
