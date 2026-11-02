// Existing HandClient only: no serial ownership and no raw command topic.
#include "rh56/hand_client.hpp"
#include <unitree/robot/channel/channel_factory.hpp>
#include <iostream>
#include <sstream>
#include <thread>
#include <chrono>
#include <cmath>
#include <cstdlib>

int main(int argc, char** argv)
{
    if (argc != 2) return 2;
    const char* config = std::getenv("CYCLONEDDS_URI");
    if (config && *config) {
        unitree::common::JsonMap parameters;
        parameters[UT_DDS_PARAM_KEY_DOMAINID] = int32_t(0);
        parameters[UT_DDS_PARAM_KEY_CONFIG] = std::string(config);
        unitree::robot::ChannelFactory::Instance()->Init(parameters);
    } else {
        unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    }
    rh56::HandClient client;
    client.SetTimeout(.6F);
    client.Init();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::string line;
    while (std::getline(std::cin, line)) {
        std::istringstream input(line);
        std::string operation, side, extra;
        uint64_t id = 0;
        input >> operation >> side >> id;
        int code = 100;
        std::string data;
        if (input && (side == "left" || side == "right") && id > 0) {
            if (operation == "get" && !(input >> extra)) {
                rh56::StateReply reply;
                code = client.GetState(side, reply, true);
                data = unitree::common::ToJsonString(reply);
            } else if (operation == "hold" && !(input >> extra)) {
                rh56::HoldRequest request; request.hand = side; request.request_id = id;
                rh56::OperationReply reply;
                code = client.Hold(request, reply);
                data = unitree::common::ToJsonString(reply);
            } else if (operation == "set") {
                rh56::SetTargetsRequest request;
                request.hand = side; request.command_id = id;
                request.joint_mask = 63; request.timeout_ms = 1000;
                bool valid = true;
                for (int i = 0; i < 6; ++i) {
                    float q = -1;
                    if (!(input >> q) || !std::isfinite(q) || q < 0 || q > 1) valid = false;
                    request.q.push_back(q);
                }
                if (input >> extra) valid = false;
                if (valid) {
                    rh56::OperationReply reply;
                    code = client.SetTargets(request, reply);
                    data = unitree::common::ToJsonString(reply);
                }
            }
        }
        if (code != 0 || data.empty()) data = "{\"code\":" + std::to_string(code) + "}";
        std::cout << "TASK_HAND " << data << std::endl;
    }
}
