#pragma once

#include "hand_api.hpp"
#include "hand_types.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <unistd.h>

namespace rh56 {

constexpr uint32_t kMaximumPoseDelayMs = 3000;

template <typename T>
T LoadJson(const std::filesystem::path& path, T fallback = {})
{
    if (!std::filesystem::exists(path))
        return fallback;
    std::ifstream input(path);
    if (!input)
        throw std::runtime_error("cannot read " + path.string());
    const std::string json{std::istreambuf_iterator<char>(input), {}};
    unitree::common::FromJsonString(json, fallback);
    return fallback;
}

template <typename T>
void StoreJson(const std::filesystem::path& path, const T& value)
{
    if (!path.parent_path().empty())
        std::filesystem::create_directories(path.parent_path());
    const auto temporary = path.string() + ".tmp-" + std::to_string(getpid());
    {
        std::ofstream output(temporary, std::ios::trunc);
        output << unitree::common::ToJsonString(value, true) << '\n';
        if (!output)
            throw std::runtime_error("cannot write " + path.string());
    }
    std::error_code error;
    std::filesystem::rename(temporary, path, error);
    if (error) {
        std::filesystem::remove(temporary);
        throw std::runtime_error("cannot replace " + path.string() + ": " +
                                 error.message());
    }
}

class PoseStore
{
public:
    explicit PoseStore(std::filesystem::path path)
        : path_(std::move(path)), poses_(LoadJson<std::vector<Pose>>(path_))
    {
        if (!std::all_of(poses_.begin(), poses_.end(),
                         [](const Pose& pose) { return Valid(pose); }))
            throw std::runtime_error("pose file contains invalid data");
    }

    std::vector<Pose> List() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return poses_;
    }

    std::optional<Pose> Find(const std::string& id) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto pose = Find(poses_, id);
        return pose == poses_.end() ? std::nullopt
                                    : std::optional<Pose>{*pose};
    }

    Pose Save(const PoseRequest& request)
    {
        Pose pose;
        pose.id = request.id;
        pose.created = NowMs();
        pose.name = request.name;
        pose.right = request.right;
        pose.left = request.left;
        pose.delays_ms = request.delays_ms.empty()
                             ? std::vector<uint32_t>(kJointCount, 0)
                             : request.delays_ms;
        if (!Valid(pose, true))
            throw std::invalid_argument("invalid pose fields");

        std::lock_guard<std::mutex> lock(mutex_);
        if (!pose.id.empty()) {
            const auto existing = Find(poses_, pose.id);
            if (existing != poses_.end())
                return *existing;
        } else {
            do {
                pose.id = std::to_string(pose.created) + '-' +
                          std::to_string(++id_counter_);
            } while (Find(poses_, pose.id) != poses_.end());
        }
        auto updated = poses_;
        updated.insert(updated.begin(), pose);
        Commit(std::move(updated));
        return pose;
    }

    std::optional<Pose> Rename(const std::string& id, const std::string& name)
    {
        if (!ValidId(id) || name.empty() || name.size() > 96)
            throw std::invalid_argument("invalid pose id or name");
        return Update(id, [&](Pose& pose) { pose.name = name; });
    }

    std::optional<Pose> SetDelays(const std::string& id,
                                  const std::vector<uint32_t>& delays)
    {
        if (!ValidId(id) || delays.size() != kJointCount ||
            !std::all_of(delays.begin(), delays.end(),
                         [](uint32_t value) {
                             return value <= kMaximumPoseDelayMs;
                         }))
            throw std::invalid_argument("expected six delays in 0..3000 ms");
        return Update(id, [&](Pose& pose) { pose.delays_ms = delays; });
    }

    bool Delete(const std::string& id)
    {
        if (!ValidId(id))
            throw std::invalid_argument("invalid pose id");
        std::lock_guard<std::mutex> lock(mutex_);
        auto updated = poses_;
        const auto pose = Find(updated, id);
        if (pose == updated.end())
            return false;
        updated.erase(pose);
        Commit(std::move(updated));
        return true;
    }

private:
    static int64_t NowMs()
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::system_clock::now().time_since_epoch())
            .count();
    }

    static bool ValidId(const std::string& id)
    {
        return !id.empty() && id.size() <= 80 &&
               std::all_of(id.begin(), id.end(), [](unsigned char byte) {
                   return std::isalnum(byte) || byte == '-';
               });
    }

    static bool Valid(const Pose& pose, bool allow_empty_id = false)
    {
        const auto valid_positions = [](const std::vector<float>& values) {
            return values.size() == kJointCount &&
                   std::all_of(values.begin(), values.end(), [](float value) {
                       return std::isfinite(value) && value >= 0.0f &&
                              value <= 1.0f;
                   });
        };
        return (ValidId(pose.id) || (allow_empty_id && pose.id.empty())) &&
               !pose.name.empty() && pose.name.size() <= 96 &&
               valid_positions(pose.right) && valid_positions(pose.left) &&
               pose.delays_ms.size() == kJointCount &&
               std::all_of(pose.delays_ms.begin(), pose.delays_ms.end(),
                           [](uint32_t value) {
                               return value <= kMaximumPoseDelayMs;
                           });
    }

    static std::vector<Pose>::iterator Find(std::vector<Pose>& poses,
                                            const std::string& id)
    {
        return std::find_if(poses.begin(), poses.end(),
                            [&](const Pose& pose) { return pose.id == id; });
    }

    static std::vector<Pose>::const_iterator Find(
        const std::vector<Pose>& poses, const std::string& id)
    {
        return std::find_if(poses.begin(), poses.end(),
                            [&](const Pose& pose) { return pose.id == id; });
    }

    template <typename Change>
    std::optional<Pose> Update(const std::string& id, Change change)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto updated = poses_;
        const auto pose = Find(updated, id);
        if (pose == updated.end())
            return std::nullopt;
        change(*pose);
        const Pose result = *pose;
        Commit(std::move(updated));
        return result;
    }

    void Commit(std::vector<Pose> updated)
    {
        StoreJson(path_, updated);
        poses_ = std::move(updated);
    }

    std::filesystem::path path_;
    mutable std::mutex mutex_;
    std::vector<Pose> poses_;
    uint64_t id_counter_{0};
};

class SettingsStore
{
public:
    explicit SettingsStore(std::filesystem::path path)
        : path_(std::move(path)), settings_(LoadJson(path_, HandSettings{}))
    {
        if (!ValidSettings(settings_))
            throw std::runtime_error("settings file contains invalid data");
    }

    HandSettings Get() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return settings_;
    }

    void Set(const HandSettings& settings)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        StoreJson(path_, settings);
        settings_ = settings;
    }

private:
    std::filesystem::path path_;
    mutable std::mutex mutex_;
    HandSettings settings_;
};

}  // namespace rh56
