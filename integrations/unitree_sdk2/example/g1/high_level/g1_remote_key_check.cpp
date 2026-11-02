#define main g1_arm_sdk_safe_probe_embedded_main
#include "g1_arm_sdk_safe_probe.cpp"
#undef main

#include <iomanip>

namespace
{
constexpr std::uint16_t kF1XMask = 0x0440;
constexpr std::uint16_t kF1AMask = 0x0140;
constexpr std::uint16_t kL2BMaskLocal = 0x0220;

std::uint16_t keys_from(const LowState& state)
{
  const auto& remote = state.wireless_remote();
  return static_cast<std::uint16_t>(remote.at(2)) |
    (static_cast<std::uint16_t>(remote.at(3)) << 8U);
}
}

int main(int argc, char** argv)
{
  if (argc != 2)
  {
    std::cerr << "Usage: " << argv[0] << " NETWORK_INTERFACE\n";
    return 1;
  }
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  SharedState shared;
  auto state_subscriber =
    std::make_unique<unitree::robot::ChannelSubscriber<LowState>>(kStateTopic);
  state_subscriber->InitChannel([&shared](const void* data) {
    std::lock_guard<std::mutex> lock(shared.mutex);
    shared.message = *static_cast<const LowState*>(data);
    shared.received_at = Clock::now();
    ++shared.samples;
  }, 1);
  auto sport_subscriber =
    std::make_unique<unitree::robot::ChannelSubscriber<SportModeState>>(kSportStateTopic);
  sport_subscriber->InitChannel([&shared](const void* data) {
    std::lock_guard<std::mutex> lock(shared.mutex);
    shared.sport_message = *static_cast<const SportModeState*>(data);
    shared.sport_received_at = Clock::now();
    ++shared.sport_samples;
  }, 1);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  std::cout << "STRICTLY_READ_ONLY: DDS subscribers only; no command publisher.\n"
            << "Test F1+X (0x0440) and F1+A (0x0140) separately. "
               "L2+B remains 0x0220. Ctrl+C exits.\n" << std::flush;
  std::uint16_t previous = 0xffff;
  std::uint8_t previous_machine = 0xff;
  std::uint32_t previous_fsm = std::numeric_limits<std::uint32_t>::max();
  while (!g_stop_requested.load())
  {
    Clock::time_point received;
    std::uint64_t count = 0;
    const auto state = snapshot(shared, received, count);
    const auto keys = keys_from(state);
    SportModeState sport;
    {
      std::lock_guard<std::mutex> lock(shared.mutex);
      sport = shared.sport_message;
    }
    if (keys != previous || state.mode_machine() != previous_machine ||
        sport.fsm_id() != previous_fsm)
    {
      std::cout << "KEY_STATE keys=0x" << std::hex << std::setw(4)
                << std::setfill('0') << keys << std::dec << std::setfill(' ')
                << " f1x=" << ((keys & kF1XMask) == kF1XMask)
                << " f1a=" << ((keys & kF1AMask) == kF1AMask)
                << " l2b=" << ((keys & kL2BMaskLocal) == kL2BMaskLocal)
                << " mode_machine=" << static_cast<int>(state.mode_machine())
                << " fsm_id=" << sport.fsm_id()
                << " fsm_mode=" << sport.fsm_mode() << '\n' << std::flush;
      previous = keys;
      previous_machine = state.mode_machine();
      previous_fsm = sport.fsm_id();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  std::cout << "Read-only key check stopped.\n";
  return 0;
}
