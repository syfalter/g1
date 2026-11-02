#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>

int main(int argc, char** argv)
{
  if (argc != 2)
  {
    std::cerr << "Usage: " << argv[0] << " NETWORK_INTERFACE" << std::endl;
    return 1;
  }

  using LowCmd = unitree_hg::msg::dds_::LowCmd_;
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  auto channel = unitree::robot::ChannelFactory::Instance()->CreateSendChannel<LowCmd>(
    "rt/arm_sdk");
  std::this_thread::sleep_for(std::chrono::seconds(3));
  // SDK2 exposes the native writer as const even though CycloneDDS's status
  // getter is not const in this bundled release. This probe does not mutate it.
  using NativeWriter = unitree::common::DdsWriter<LowCmd>::NATIVE_TYPE;
  auto& native_writer = const_cast<NativeWriter&>(channel->GetWriter()->GetNative());
  const auto status = native_writer.publication_matched_status();
  std::cout << "rt/arm_sdk publication matched: current_count="
            << status.current_count() << " total_count=" << status.total_count()
            << " current_count_change=" << status.current_count_change()
            << " total_count_change=" << status.total_count_change() << std::endl;
  std::cout << "No LowCmd sample was written." << std::endl;
  return status.current_count() > 0 ? 0 : 2;
}
