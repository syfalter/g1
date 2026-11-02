#pragma once

#include <cstdint>
#include <string>
#include <unitree/common/json/jsonize.hpp>
#include <vector>

namespace vision {

inline const std::string kServiceName = "vision_service";
inline const std::string kApiVersion = "1.0.0.0";
constexpr int32_t kApiRecognizeOnce = 2001;
constexpr int32_t kApiGetStatus = 2002;

enum class ResultCode : int32_t {
    kOk = 0,
    kInvalidArgument = 100,
    kWorkerUnavailable = 101,
    kNoTarget = 102,
    kInvalidDepth = 103,
    kTargetFrameUnavailable = 104,
    kWorkerError = 200,
    kInvalidResponse = 201,
};

class RecognizeRequest : public unitree::common::Jsonize
{
public:
    uint64_t request_id{0};
    bool require_3d{true};
    bool require_target_frame{false};
    bool save_artifacts{false};
    bool restart_stream{false};

    void fromJson(unitree::common::JsonMap& json) override
    {
        JN_FROM(json, "request_id", request_id);
        JN_FROM_WEAK(json, "require_3d", require_3d);
        JN_FROM_WEAK(json, "require_target_frame", require_target_frame);
        JN_FROM_WEAK(json, "save_artifacts", save_artifacts);
        JN_FROM_WEAK(json, "restart_stream", restart_stream);
    }

    void toJson(unitree::common::JsonMap& json) const override
    {
        JN_TO(json, "request_id", request_id);
        JN_TO(json, "require_3d", require_3d);
        JN_TO(json, "require_target_frame", require_target_frame);
        JN_TO(json, "save_artifacts", save_artifacts);
        JN_TO(json, "restart_stream", restart_stream);
    }
};

class StatusRequest : public unitree::common::Jsonize
{
public:
    uint64_t request_id{0};

    void fromJson(unitree::common::JsonMap& json) override
    {
        JN_FROM(json, "request_id", request_id);
    }

    void toJson(unitree::common::JsonMap& json) const override
    {
        JN_TO(json, "request_id", request_id);
    }
};

class PointSet3D : public unitree::common::Jsonize
{
public:
    std::vector<double> black_center_point;
    std::vector<double> operation_diagonal_intersection;
    std::vector<double> operation_point;

    void fromJson(unitree::common::JsonMap& json) override
    {
        JN_FROM_WEAK(json, "black_center_point", black_center_point);
        JN_FROM_WEAK(json, "operation_diagonal_intersection", operation_diagonal_intersection);
        JN_FROM_WEAK(json, "operation_point", operation_point);
    }

    void toJson(unitree::common::JsonMap& json) const override
    {
        JN_TO(json, "black_center_point", black_center_point);
        JN_TO(json, "operation_diagonal_intersection", operation_diagonal_intersection);
        JN_TO(json, "operation_point", operation_point);
    }
};

class CameraIntrinsics : public unitree::common::Jsonize
{
public:
    double fx{0.0};
    double fy{0.0};
    double cx{0.0};
    double cy{0.0};
    int32_t width{0};
    int32_t height{0};
    std::string distortion_model;
    std::vector<double> distortion;

    void fromJson(unitree::common::JsonMap& json) override
    {
        JN_FROM_WEAK(json, "fx", fx); JN_FROM_WEAK(json, "fy", fy);
        JN_FROM_WEAK(json, "cx", cx); JN_FROM_WEAK(json, "cy", cy);
        JN_FROM_WEAK(json, "width", width); JN_FROM_WEAK(json, "height", height);
        JN_FROM_WEAK(json, "distortion_model", distortion_model);
        JN_FROM_WEAK(json, "distortion", distortion);
    }

    void toJson(unitree::common::JsonMap& json) const override
    {
        JN_TO(json, "fx", fx); JN_TO(json, "fy", fy);
        JN_TO(json, "cx", cx); JN_TO(json, "cy", cy);
        JN_TO(json, "width", width); JN_TO(json, "height", height);
        JN_TO(json, "distortion_model", distortion_model);
        JN_TO(json, "distortion", distortion);
    }
};

class SensorExtrinsics : public unitree::common::Jsonize
{
public:
    std::vector<double> rotation;
    std::vector<double> translation_m;

    void fromJson(unitree::common::JsonMap& json) override
    {
        JN_FROM_WEAK(json, "rotation", rotation);
        JN_FROM_WEAK(json, "translation_m", translation_m);
    }

    void toJson(unitree::common::JsonMap& json) const override
    {
        JN_TO(json, "rotation", rotation);
        JN_TO(json, "translation_m", translation_m);
    }
};

class ObservationReply : public unitree::common::Jsonize
{
public:
    int32_t code{0};
    std::string message{"ok"};
    uint64_t request_id{0};
    uint64_t sequence{0};
    double captured_at_ms{0.0};
    double inference_ms{0.0};
    bool valid_2d{false};
    bool valid_3d{false};
    bool target_frame_ready{false};
    int32_t image_width{0};
    int32_t image_height{0};
    std::vector<int32_t> pedal_centered;
    std::vector<int32_t> black_center;
    std::vector<int32_t> black_center_point;
    std::vector<std::vector<int32_t>> operation_zone;
    std::vector<int32_t> operation_diagonal_intersection;
    std::vector<int32_t> operation_point;
    double stage1_confidence{-1.0};
    double stage2_confidence{-1.0};
    PointSet3D points_camera;
    PointSet3D points_target;
    std::string camera_frame;
    std::string target_frame;
    std::string transform_status;
    std::string units;
    std::vector<std::string> invalid_depth_keys;
    std::string camera_name;
    std::string camera_serial;
    double depth_scale_m_per_unit{0.0};
    uint64_t color_frame_number{0};
    uint64_t depth_frame_number{0};
    double color_timestamp_ms{0.0};
    double depth_timestamp_ms{0.0};
    double timestamp_delta_ms{0.0};
    CameraIntrinsics color_intrinsics;
    SensorExtrinsics depth_to_color_extrinsics;
    std::string capture_result;

    void fromJson(unitree::common::JsonMap& json) override;
    void toJson(unitree::common::JsonMap& json) const override;
};

class StatusReply : public unitree::common::Jsonize
{
public:
    int32_t code{0};
    std::string message{"ok"};
    uint64_t request_id{0};
    std::string state;
    bool camera_ready{false};
    bool model_ready{false};
    bool calibration_configured{false};
    std::string camera_frame;
    std::string target_frame;
    std::string last_error;

    void fromJson(unitree::common::JsonMap& json) override
    {
        JN_FROM_WEAK(json, "code", code); JN_FROM_WEAK(json, "message", message);
        JN_FROM_WEAK(json, "request_id", request_id); JN_FROM_WEAK(json, "state", state);
        JN_FROM_WEAK(json, "camera_ready", camera_ready); JN_FROM_WEAK(json, "model_ready", model_ready);
        JN_FROM_WEAK(json, "calibration_configured", calibration_configured);
        JN_FROM_WEAK(json, "camera_frame", camera_frame); JN_FROM_WEAK(json, "target_frame", target_frame);
        JN_FROM_WEAK(json, "last_error", last_error);
    }

    void toJson(unitree::common::JsonMap& json) const override
    {
        JN_TO(json, "code", code); JN_TO(json, "message", message);
        JN_TO(json, "request_id", request_id); JN_TO(json, "state", state);
        JN_TO(json, "camera_ready", camera_ready); JN_TO(json, "model_ready", model_ready);
        JN_TO(json, "calibration_configured", calibration_configured);
        JN_TO(json, "camera_frame", camera_frame); JN_TO(json, "target_frame", target_frame);
        JN_TO(json, "last_error", last_error);
    }
};

inline void ObservationReply::fromJson(unitree::common::JsonMap& json)
{
#define VISION_FROM(field) JN_FROM_WEAK(json, #field, field)
    VISION_FROM(code); VISION_FROM(message); VISION_FROM(request_id); VISION_FROM(sequence);
    VISION_FROM(captured_at_ms); VISION_FROM(inference_ms); VISION_FROM(valid_2d);
    VISION_FROM(valid_3d); VISION_FROM(target_frame_ready); VISION_FROM(image_width);
    VISION_FROM(image_height); VISION_FROM(pedal_centered); VISION_FROM(black_center);
    VISION_FROM(black_center_point); VISION_FROM(operation_zone);
    VISION_FROM(operation_diagonal_intersection); VISION_FROM(operation_point);
    VISION_FROM(stage1_confidence); VISION_FROM(stage2_confidence); VISION_FROM(points_camera);
    VISION_FROM(points_target); VISION_FROM(camera_frame); VISION_FROM(target_frame);
    VISION_FROM(transform_status); VISION_FROM(units); VISION_FROM(invalid_depth_keys);
    VISION_FROM(camera_name); VISION_FROM(camera_serial); VISION_FROM(depth_scale_m_per_unit);
    VISION_FROM(color_frame_number); VISION_FROM(depth_frame_number); VISION_FROM(color_timestamp_ms);
    VISION_FROM(depth_timestamp_ms); VISION_FROM(timestamp_delta_ms); VISION_FROM(color_intrinsics);
    VISION_FROM(depth_to_color_extrinsics); VISION_FROM(capture_result);
#undef VISION_FROM
}

inline void ObservationReply::toJson(unitree::common::JsonMap& json) const
{
#define VISION_TO(field) JN_TO(json, #field, field)
    VISION_TO(code); VISION_TO(message); VISION_TO(request_id); VISION_TO(sequence);
    VISION_TO(captured_at_ms); VISION_TO(inference_ms); VISION_TO(valid_2d);
    VISION_TO(valid_3d); VISION_TO(target_frame_ready); VISION_TO(image_width);
    VISION_TO(image_height); VISION_TO(pedal_centered); VISION_TO(black_center);
    VISION_TO(black_center_point); VISION_TO(operation_zone);
    VISION_TO(operation_diagonal_intersection); VISION_TO(operation_point);
    VISION_TO(stage1_confidence); VISION_TO(stage2_confidence); VISION_TO(points_camera);
    VISION_TO(points_target); VISION_TO(camera_frame); VISION_TO(target_frame);
    VISION_TO(transform_status); VISION_TO(units); VISION_TO(invalid_depth_keys);
    VISION_TO(camera_name); VISION_TO(camera_serial); VISION_TO(depth_scale_m_per_unit);
    VISION_TO(color_frame_number); VISION_TO(depth_frame_number); VISION_TO(color_timestamp_ms);
    VISION_TO(depth_timestamp_ms); VISION_TO(timestamp_delta_ms); VISION_TO(color_intrinsics);
    VISION_TO(depth_to_color_extrinsics); VISION_TO(capture_result);
#undef VISION_TO
}

}  // namespace vision
