#include "vision/vision_api.hpp"

#include <iostream>

int main()
{
    vision::RecognizeRequest request;
    request.request_id = 42;
    request.require_3d = true;
    request.require_target_frame = false;
    request.save_artifacts = true;
    vision::RecognizeRequest decoded_request;
    unitree::common::FromJsonString(
        unitree::common::ToJsonString(request), decoded_request);
    if (decoded_request.request_id != 42 || !decoded_request.require_3d ||
        decoded_request.require_target_frame || !decoded_request.save_artifacts) {
        std::cerr << "RecognizeRequest JSON round trip failed\n";
        return 1;
    }

    vision::ObservationReply reply;
    reply.code = 0;
    reply.request_id = 42;
    reply.sequence = 81;
    reply.valid_2d = true;
    reply.valid_3d = true;
    reply.operation_point = {640, 380};
    reply.points_camera.operation_point = {0.01, 0.08, 0.53};
    reply.transform_status = "not_configured";
    reply.camera_frame = "head_color_camera_optical_frame";
    vision::ObservationReply decoded_reply;
    unitree::common::FromJsonString(
        unitree::common::ToJsonString(reply), decoded_reply);
    if (decoded_reply.request_id != 42 || decoded_reply.sequence != 81 ||
        !decoded_reply.valid_3d || decoded_reply.operation_point != reply.operation_point ||
        decoded_reply.points_camera.operation_point != reply.points_camera.operation_point ||
        decoded_reply.transform_status != "not_configured") {
        std::cerr << "ObservationReply JSON round trip failed\n";
        return 1;
    }

    vision::StatusReply status;
    status.state = "idle";
    status.camera_ready = true;
    status.model_ready = true;
    vision::StatusReply decoded_status;
    unitree::common::FromJsonString(
        unitree::common::ToJsonString(status), decoded_status);
    if (decoded_status.state != "idle" || !decoded_status.camera_ready ||
        !decoded_status.model_ready) {
        std::cerr << "StatusReply JSON round trip failed\n";
        return 1;
    }
    return 0;
}
