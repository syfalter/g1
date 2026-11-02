#pragma once

#include "vision_api.hpp"

#include <unitree/robot/client/client.hpp>

namespace vision {

class VisionClient : public unitree::robot::Client
{
public:
    VisionClient() : Client(kServiceName, false) {}

    void Init() override
    {
        SetApiVersion(kApiVersion);
        UT_ROBOT_CLIENT_REG_API_NO_PROI(kApiRecognizeOnce);
        UT_ROBOT_CLIENT_REG_API_NO_PROI(kApiGetStatus);
    }

    int32_t RecognizeOnce(const RecognizeRequest& request, ObservationReply& reply)
    {
        return CallAndDecode(kApiRecognizeOnce, request, reply);
    }

    int32_t GetStatus(uint64_t request_id, StatusReply& reply)
    {
        StatusRequest request;
        request.request_id = request_id;
        return CallAndDecode(kApiGetStatus, request, reply);
    }

private:
    template <typename Request, typename Reply>
    int32_t CallAndDecode(int32_t api_id, const Request& request, Reply& reply)
    {
        std::string data;
        const int32_t transport_code =
            Call(api_id, unitree::common::ToJsonString(request), data);
        if (transport_code != 0)
            return transport_code;
        try {
            unitree::common::FromJsonString(data, reply);
            return reply.code;
        } catch (...) {
            return static_cast<int32_t>(ResultCode::kInvalidResponse);
        }
    }
};

}  // namespace vision
