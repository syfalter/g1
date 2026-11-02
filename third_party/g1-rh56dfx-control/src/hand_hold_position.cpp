// Best-effort software hold through the unified RH56 service.

#include "rh56/hand_client.hpp"

#include <unitree/robot/channel/channel_factory.hpp>

#include <chrono>
#include <iostream>
#include <string>

namespace {

bool Hold(rh56::HandClient& client, const std::string& hand, uint64_t request_id)
{
    rh56::HoldRequest request;
    request.hand = hand;
    request.request_id = request_id;
    rh56::OperationReply reply;
    const int32_t result = client.Hold(request, reply);
    if (result != 0) {
        std::cerr << hand << ": "
                  << (reply.message.empty() ? "hold RPC failed" : reply.message)
                  << " (code=" << result << ")\n";
        return false;
    }
    std::cout << hand << ": holding current position\n";
    return true;
}

}  // namespace

int main(int argc, char** argv)
{
    std::string selection{"both"};
    std::string network;
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if (argument == "--network" && i + 1 < argc)
            network = argv[++i];
        else if (argument == "left" || argument == "right" || argument == "both")
            selection = argument;
        else {
            std::cerr << "Usage: " << argv[0]
                      << " [left|right|both] [--network INTERFACE]\n";
            return 2;
        }
    }

    try {
        unitree::robot::ChannelFactory::Instance()->Init(0, network);
        rh56::HandClient client;
        client.SetTimeout(1.0f);
        client.Init();
        const uint64_t request_id = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
        bool ok = true;
        if (selection == "left" || selection == "both")
            ok = Hold(client, "left", request_id) && ok;
        if (selection == "right" || selection == "both")
            ok = Hold(client, "right", request_id + 1) && ok;
        return ok ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
