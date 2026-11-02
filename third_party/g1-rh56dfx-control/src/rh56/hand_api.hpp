#pragma once

#include <cstdint>
#include <string>
#include <unitree/common/json/jsonize.hpp>
#include <vector>

namespace rh56 {

inline const std::string kServiceName = "rh56_hand";
inline const std::string kApiVersion = "1.3.0.0";
constexpr int32_t kApiSetTargets = 1001;
constexpr int32_t kApiClearFault = 1002;
constexpr int32_t kApiGetState = 1003;
constexpr int32_t kApiApplyGrip = 1004;
constexpr int32_t kApiHold = 1005;
constexpr int32_t kApiPoses = 1006;
constexpr int32_t kApiSettings = 1007;

class SetTargetsRequest : public unitree::common::Jsonize
{
public:
    std::string hand;
    uint32_t joint_mask{0};
    std::vector<float> q;
    uint64_t command_id{0};
    uint32_t timeout_ms{300};

    void fromJson(unitree::common::JsonMap& json) override
    {
        JN_FROM(json, "hand", hand);
        JN_FROM(json, "joint_mask", joint_mask);
        JN_FROM(json, "q", q);
        JN_FROM(json, "command_id", command_id);
        JN_FROM_WEAK(json, "timeout_ms", timeout_ms);
    }

    void toJson(unitree::common::JsonMap& json) const override
    {
        JN_TO(json, "hand", hand);
        JN_TO(json, "joint_mask", joint_mask);
        JN_TO(json, "q", q);
        JN_TO(json, "command_id", command_id);
        JN_TO(json, "timeout_ms", timeout_ms);
    }
};

class ClearFaultRequest : public unitree::common::Jsonize
{
public:
    std::string hand;
    uint32_t joint_mask{0};
    uint64_t request_id{0};

    void fromJson(unitree::common::JsonMap& json) override
    {
        JN_FROM(json, "hand", hand);
        JN_FROM(json, "joint_mask", joint_mask);
        JN_FROM(json, "request_id", request_id);
    }

    void toJson(unitree::common::JsonMap& json) const override
    {
        JN_TO(json, "hand", hand);
        JN_TO(json, "joint_mask", joint_mask);
        JN_TO(json, "request_id", request_id);
    }
};

class GripRequest : public unitree::common::Jsonize
{
public:
    std::string hand;
    std::vector<float> q;
    int32_t force_grams{0};
    int32_t current_ma{0};
    uint64_t command_id{0};
    uint32_t timeout_ms{300};

    void fromJson(unitree::common::JsonMap& json) override
    {
        JN_FROM(json, "hand", hand);
        JN_FROM(json, "q", q);
        JN_FROM(json, "force_grams", force_grams);
        JN_FROM(json, "current_ma", current_ma);
        JN_FROM(json, "command_id", command_id);
        JN_FROM_WEAK(json, "timeout_ms", timeout_ms);
    }

    void toJson(unitree::common::JsonMap& json) const override
    {
        JN_TO(json, "hand", hand);
        JN_TO(json, "q", q);
        JN_TO(json, "force_grams", force_grams);
        JN_TO(json, "current_ma", current_ma);
        JN_TO(json, "command_id", command_id);
        JN_TO(json, "timeout_ms", timeout_ms);
    }
};

class GetStateRequest : public unitree::common::Jsonize
{
public:
    std::string hand;
    bool refresh{false};

    void fromJson(unitree::common::JsonMap& json) override
    {
        JN_FROM(json, "hand", hand);
        JN_FROM_WEAK(json, "refresh", refresh);
    }

    void toJson(unitree::common::JsonMap& json) const override
    {
        JN_TO(json, "hand", hand);
        JN_TO(json, "refresh", refresh);
    }
};

class HoldRequest : public unitree::common::Jsonize
{
public:
    std::string hand;
    uint64_t request_id{0};

    void fromJson(unitree::common::JsonMap& json) override
    {
        JN_FROM(json, "hand", hand);
        JN_FROM(json, "request_id", request_id);
    }

    void toJson(unitree::common::JsonMap& json) const override
    {
        JN_TO(json, "hand", hand);
        JN_TO(json, "request_id", request_id);
    }
};

class Pose : public unitree::common::Jsonize
{
public:
    std::string id;
    int64_t created{0};
    std::string name;
    std::vector<float> right;
    std::vector<float> left;
    std::vector<uint32_t> delays_ms;

    void fromJson(unitree::common::JsonMap& json) override
    {
        JN_FROM(json, "id", id);
        JN_FROM(json, "created", created);
        JN_FROM(json, "name", name);
        JN_FROM(json, "right", right);
        JN_FROM(json, "left", left);
        JN_FROM(json, "delays_ms", delays_ms);
    }

    void toJson(unitree::common::JsonMap& json) const override
    {
        JN_TO(json, "id", id);
        JN_TO(json, "created", created);
        JN_TO(json, "name", name);
        JN_TO(json, "right", right);
        JN_TO(json, "left", left);
        JN_TO(json, "delays_ms", delays_ms);
    }
};

class PoseRequest : public unitree::common::Jsonize
{
public:
    std::string action;
    std::string id;
    std::string name;
    std::vector<float> right;
    std::vector<float> left;
    std::vector<uint32_t> delays_ms;
    std::string hand{"both"};
    uint64_t request_id{0};
    uint32_t timeout_ms{5000};

    void fromJson(unitree::common::JsonMap& json) override
    {
        JN_FROM(json, "action", action);
        JN_FROM_WEAK(json, "id", id);
        JN_FROM_WEAK(json, "name", name);
        JN_FROM_WEAK(json, "right", right);
        JN_FROM_WEAK(json, "left", left);
        JN_FROM_WEAK(json, "delays_ms", delays_ms);
        JN_FROM_WEAK(json, "hand", hand);
        JN_FROM_WEAK(json, "request_id", request_id);
        JN_FROM_WEAK(json, "timeout_ms", timeout_ms);
    }

    void toJson(unitree::common::JsonMap& json) const override
    {
        JN_TO(json, "action", action);
        JN_TO(json, "id", id);
        JN_TO(json, "name", name);
        JN_TO(json, "right", right);
        JN_TO(json, "left", left);
        JN_TO(json, "delays_ms", delays_ms);
        JN_TO(json, "hand", hand);
        JN_TO(json, "request_id", request_id);
        JN_TO(json, "timeout_ms", timeout_ms);
    }
};

class HandSettings : public unitree::common::Jsonize
{
public:
    int32_t right_grip_force_grams{300};
    int32_t right_grip_current_ma{200};
    int32_t left_grip_force_grams{300};
    int32_t left_grip_current_ma{200};
    int32_t contact_threshold{50};

    void fromJson(unitree::common::JsonMap& json) override
    {
        JN_FROM(json, "right_grip_force_grams", right_grip_force_grams);
        JN_FROM(json, "right_grip_current_ma", right_grip_current_ma);
        JN_FROM(json, "left_grip_force_grams", left_grip_force_grams);
        JN_FROM(json, "left_grip_current_ma", left_grip_current_ma);
        JN_FROM(json, "contact_threshold", contact_threshold);
    }

    void toJson(unitree::common::JsonMap& json) const override
    {
        JN_TO(json, "right_grip_force_grams", right_grip_force_grams);
        JN_TO(json, "right_grip_current_ma", right_grip_current_ma);
        JN_TO(json, "left_grip_force_grams", left_grip_force_grams);
        JN_TO(json, "left_grip_current_ma", left_grip_current_ma);
        JN_TO(json, "contact_threshold", contact_threshold);
    }
};

inline bool ValidSettings(const HandSettings& value)
{
    return value.right_grip_force_grams >= 50 &&
           value.right_grip_force_grams <= 1000 &&
           value.left_grip_force_grams >= 50 &&
           value.left_grip_force_grams <= 1000 &&
           value.right_grip_current_ma >= 50 &&
           value.right_grip_current_ma <= 300 &&
           value.left_grip_current_ma >= 50 &&
           value.left_grip_current_ma <= 300 &&
           value.contact_threshold >= 1 && value.contact_threshold <= 1000;
}

class SettingsMessage : public unitree::common::Jsonize
{
public:
    int32_t code{0};
    std::string message;
    bool write{false};
    HandSettings settings;
    uint64_t request_id{0};

    void fromJson(unitree::common::JsonMap& json) override
    {
        JN_FROM_WEAK(json, "code", code);
        JN_FROM_WEAK(json, "message", message);
        JN_FROM_WEAK(json, "write", write);
        JN_FROM_WEAK(json, "settings", settings);
        JN_FROM_WEAK(json, "request_id", request_id);
    }

    void toJson(unitree::common::JsonMap& json) const override
    {
        JN_TO(json, "code", code);
        JN_TO(json, "message", message);
        JN_TO(json, "write", write);
        JN_TO(json, "settings", settings);
        JN_TO(json, "request_id", request_id);
    }
};

class OperationReply : public unitree::common::Jsonize
{
public:
    int32_t code{0};
    std::string message;
    uint64_t request_id{0};
    uint32_t affected_mask{0};
    std::vector<int32_t> error;
    std::vector<int32_t> status;
    std::vector<int32_t> temperature;

    void fromJson(unitree::common::JsonMap& json) override
    {
        JN_FROM(json, "code", code);
        JN_FROM(json, "message", message);
        JN_FROM(json, "request_id", request_id);
        JN_FROM(json, "affected_mask", affected_mask);
        JN_FROM(json, "error", error);
        JN_FROM(json, "status", status);
        JN_FROM(json, "temperature", temperature);
    }

    void toJson(unitree::common::JsonMap& json) const override
    {
        JN_TO(json, "code", code);
        JN_TO(json, "message", message);
        JN_TO(json, "request_id", request_id);
        JN_TO(json, "affected_mask", affected_mask);
        JN_TO(json, "error", error);
        JN_TO(json, "status", status);
        JN_TO(json, "temperature", temperature);
    }
};

class StateReply : public unitree::common::Jsonize
{
public:
    int32_t code{0};
    std::string message;
    std::string hand;
    bool online{false};
    std::vector<float> feedback_q;
    std::vector<float> target_q;
    std::vector<int32_t> force;
    std::vector<int32_t> current;
    std::vector<int32_t> force_limit;
    std::vector<int32_t> current_limit;
    std::vector<bool> contact;
    bool contact_monitoring{false};
    std::vector<int32_t> error;
    std::vector<int32_t> status;
    std::vector<int32_t> temperature;
    uint32_t lost_count{0};
    uint64_t last_command_id{0};
    uint64_t timestamp_ms{0};

    void fromJson(unitree::common::JsonMap& json) override
    {
        JN_FROM(json, "code", code);
        JN_FROM(json, "message", message);
        JN_FROM(json, "hand", hand);
        JN_FROM(json, "online", online);
        JN_FROM(json, "feedback_q", feedback_q);
        JN_FROM(json, "target_q", target_q);
        JN_FROM(json, "force", force);
        JN_FROM(json, "current", current);
        JN_FROM(json, "force_limit", force_limit);
        JN_FROM(json, "current_limit", current_limit);
        JN_FROM_WEAK(json, "contact", contact);
        JN_FROM_WEAK(json, "contact_monitoring", contact_monitoring);
        JN_FROM(json, "error", error);
        JN_FROM(json, "status", status);
        JN_FROM(json, "temperature", temperature);
        JN_FROM(json, "lost_count", lost_count);
        JN_FROM(json, "last_command_id", last_command_id);
        JN_FROM(json, "timestamp_ms", timestamp_ms);
    }

    void toJson(unitree::common::JsonMap& json) const override
    {
        JN_TO(json, "code", code);
        JN_TO(json, "message", message);
        JN_TO(json, "hand", hand);
        JN_TO(json, "online", online);
        JN_TO(json, "feedback_q", feedback_q);
        JN_TO(json, "target_q", target_q);
        JN_TO(json, "force", force);
        JN_TO(json, "current", current);
        JN_TO(json, "force_limit", force_limit);
        JN_TO(json, "current_limit", current_limit);
        JN_TO(json, "contact", contact);
        JN_TO(json, "contact_monitoring", contact_monitoring);
        JN_TO(json, "error", error);
        JN_TO(json, "status", status);
        JN_TO(json, "temperature", temperature);
        JN_TO(json, "lost_count", lost_count);
        JN_TO(json, "last_command_id", last_command_id);
        JN_TO(json, "timestamp_ms", timestamp_ms);
    }
};

class PoseReply : public unitree::common::Jsonize
{
public:
    int32_t code{0};
    std::string message;
    uint64_t request_id{0};
    Pose pose;
    std::vector<Pose> poses;
    uint32_t duration_ms{0};
    std::vector<std::string> affected_hands;

    void fromJson(unitree::common::JsonMap& json) override
    {
        JN_FROM(json, "code", code);
        JN_FROM(json, "message", message);
        JN_FROM_WEAK(json, "request_id", request_id);
        JN_FROM_WEAK(json, "pose", pose);
        JN_FROM_WEAK(json, "poses", poses);
        JN_FROM_WEAK(json, "duration_ms", duration_ms);
        JN_FROM_WEAK(json, "affected_hands", affected_hands);
    }

    void toJson(unitree::common::JsonMap& json) const override
    {
        JN_TO(json, "code", code);
        JN_TO(json, "message", message);
        JN_TO(json, "request_id", request_id);
        JN_TO(json, "pose", pose);
        JN_TO(json, "poses", poses);
        JN_TO(json, "duration_ms", duration_ms);
        JN_TO(json, "affected_hands", affected_hands);
    }
};

}  // namespace rh56
