#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <thread>

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace
{
using LowCmd = unitree_hg::msg::dds_::LowCmd_;
using LowState = unitree_hg::msg::dds_::LowState_;
constexpr std::uint16_t kL2BMask = 0x0220;
constexpr std::size_t kWeightIndex = 29;
}

int main(int argc, char** argv)
{
  if (argc != 3 || std::string(argv[2]) != "CLEAR_ARM_SDK_WEIGHT")
  {
    std::cerr << "Usage: " << argv[0]
              << " NETWORK_INTERFACE CLEAR_ARM_SDK_WEIGHT\n";
    return 1;
  }
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  std::atomic_bool l2_b{false};
  std::atomic<std::uint64_t> samples{0};
  auto subscriber = std::make_unique<unitree::robot::ChannelSubscriber<LowState>>(
    "rt/lowstate");
  subscriber->InitChannel([&](const void* data) {
    const auto& state = *static_cast<const LowState*>(data);
    const auto& remote = state.wireless_remote();
    const auto keys = static_cast<std::uint16_t>(remote[2]) |
      (static_cast<std::uint16_t>(remote[3]) << 8U);
    l2_b.store((keys & kL2BMask) == kL2BMask);
    ++samples;
  }, 1);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  if (samples.load() == 0 || !l2_b.load())
  {
    std::cerr << "REFUSED: fresh LowState with native L2+B is required; "
              << "no publisher created.\n";
    return 2;
  }
  auto publisher = std::make_unique<unitree::robot::ChannelPublisher<LowCmd>>(
    "rt/arm_sdk");
  publisher->InitChannel();
  LowCmd command;
  command.motor_cmd().at(kWeightIndex).q(0.0F);
  for (int i = 0; i < 10; ++i)
  {
    if (!l2_b.load())
    {
      std::cerr << "L2+B disappeared; stopped after " << i << " clear frames.\n";
      return 3;
    }
    publisher->Write(command);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  std::cout << "CLEARED: sent 10 arm_sdk frames with weight=0 while L2+B active; "
            << "publisher stopped.\n";
  return 0;
}
