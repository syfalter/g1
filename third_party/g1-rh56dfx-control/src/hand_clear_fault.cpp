#include "rh56/hand_client.hpp"

#include <unitree/robot/channel/channel_factory.hpp>

#include <array>
#include <chrono>
#include <iostream>
#include <optional>
#include <string>

namespace {

constexpr std::array<const char*, 6> kJointNames{
    "pinky", "ring", "middle", "index", "thumb-bend", "thumb-rotation"};

std::optional<std::size_t> JointIndex(const std::string& name)
{
    for (std::size_t i = 0; i < kJointNames.size(); ++i)
        if (name == kJointNames[i])
            return i;
    return std::nullopt;
}

void Usage(const char* executable)
{
    std::cerr << "Usage: " << executable
              << " --hand left|right --joint NAME --execute "
                 "[--network INTERFACE]\n";
}

}  // namespace

int main(int argc, char** argv)
{
    std::string hand;
    std::string joint;
    std::string network;
    bool execute = false;
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if (argument == "--execute") {
            execute = true;
        } else if ((argument == "--hand" || argument == "--joint" ||
                    argument == "--network") && i + 1 < argc) {
            const std::string value = argv[++i];
            if (argument == "--hand")
                hand = value;
            else if (argument == "--joint")
                joint = value;
            else
                network = value;
        } else {
            Usage(argv[0]);
            return 2;
        }
    }

    const auto joint_index = JointIndex(joint);
    if (!execute || (hand != "left" && hand != "right") || !joint_index) {
        Usage(argv[0]);
        return 2;
    }

    try {
        unitree::robot::ChannelFactory::Instance()->Init(0, network);
        rh56::HandClient client;
        client.SetTimeout(2.0f);
        client.Init();

        rh56::ClearFaultRequest request;
        request.hand = hand;
        request.joint_mask = uint32_t{1} << *joint_index;
        request.request_id = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
        rh56::OperationReply reply;
        const int32_t result = client.ClearFault(request, reply);

        const std::string message = reply.message.empty()
                                        ? "RPC transport failed"
                                        : reply.message;
        std::cout << hand << ' ' << joint << ": " << message
                  << " (code=" << result << ", affected_mask="
                  << reply.affected_mask << ")\n";
        return result == 0 ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
