#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/SportModeState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/g1/arm/g1_arm_action_client.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/b2/robot_state/robot_state_client.hpp>
#include <unitree/robot/go2/robot_state/robot_state_client.hpp>

namespace
{
using LowState = unitree_hg::msg::dds_::LowState_;
using SportModeState = unitree_hg::msg::dds_::SportModeState_;

struct State
{
  std::mutex mutex;
  LowState low;
  SportModeState sport;
  std::uint64_t low_samples = 0;
  std::uint64_t sport_samples = 0;
};
}  // namespace

int main(int argc, char** argv)
{
  if (argc != 2)
  {
    std::cerr << "Usage: " << argv[0] << " NETWORK_INTERFACE" << std::endl;
    return 1;
  }

  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  State state;
  auto low_subscriber = std::make_unique<unitree::robot::ChannelSubscriber<LowState>>(
    "rt/lowstate");
  low_subscriber->InitChannel([&state](const void* data) {
    std::lock_guard<std::mutex> lock(state.mutex);
    state.low = *static_cast<const LowState*>(data);
    ++state.low_samples;
  }, 1);
  auto sport_subscriber =
    std::make_unique<unitree::robot::ChannelSubscriber<SportModeState>>(
      "rt/sportmodestate");
  sport_subscriber->InitChannel([&state](const void* data) {
    std::lock_guard<std::mutex> lock(state.mutex);
    state.sport = *static_cast<const SportModeState*>(data);
    ++state.sport_samples;
  }, 1);

  std::this_thread::sleep_for(std::chrono::seconds(3));
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    std::cout << "LowState samples=" << state.low_samples
              << " version=[" << state.low.version()[0] << ',' << state.low.version()[1] << ']'
              << " mode_pr=" << static_cast<int>(state.low.mode_pr())
              << " mode_machine=" << static_cast<int>(state.low.mode_machine())
              << " tick=" << state.low.tick() << std::endl;
    if (state.sport_samples == 0)
      std::cout << "SportModeState samples=0 (rt/sportmodestate is not publishing)" << std::endl;
    else
      std::cout << "SportModeState samples=" << state.sport_samples
                << " fsm_id=" << state.sport.fsm_id()
                << " fsm_mode=" << state.sport.fsm_mode()
                << " task_id=" << state.sport.task_id()
                << " task_time=" << state.sport.task_time() << std::endl;
  }

  unitree::robot::go2::RobotStateClient robot_state;
  robot_state.SetTimeout(3.0F);
  robot_state.Init();
  std::cout << "RobotState client API=" << robot_state.GetApiVersion()
            << " server API=" << robot_state.GetServerApiVersion() << std::endl;
  std::vector<unitree::robot::go2::ServiceState> services;
  const auto service_result = robot_state.ServiceList(services);
  std::cout << "ServiceList result=" << service_result
            << " count=" << services.size() << std::endl;
  for (const auto& service : services)
    std::cout << "  " << service.name << " status=" << service.status
              << " protect=" << service.protect << std::endl;

  // API 1006 is exposed by the newer RobotState client variant and is read-only.
  unitree::robot::b2::RobotStateClient package_client;
  package_client.SetTimeout(3.0F);
  package_client.Init();
  std::string package_version;
  std::map<std::string, std::string> module_versions;
  const auto package_result = package_client.GetPkgVersion(
    package_version, module_versions);
  std::cout << "GetPkgVersion result=" << package_result
            << " packageVersion=" << package_version
            << " moduleCount=" << module_versions.size() << std::endl;
  for (const auto& module : module_versions)
    std::cout << "  module " << module.first << '=' << module.second << std::endl;

  unitree::robot::g1::LocoClient loco;
  loco.SetTimeout(3.0F);
  loco.Init();
  const auto sport_server_version = loco.GetServerApiVersion();
  int fsm_id = -1;
  int fsm_mode = -1;
  const auto fsm_id_result = loco.GetFsmId(fsm_id);
  const auto fsm_mode_result = loco.GetFsmMode(fsm_mode);
  std::cout << "Sport client API=" << loco.GetApiVersion()
            << " server API=" << sport_server_version
            << " GetFsmId result=" << fsm_id_result << " value=" << fsm_id
            << " GetFsmMode result=" << fsm_mode_result << " value=" << fsm_mode
            << std::endl;

  unitree::robot::b2::MotionSwitcherClient motion_switcher;
  motion_switcher.SetTimeout(3.0F);
  motion_switcher.Init();
  std::string robot_form;
  std::string motion_name;
  const auto mode_result = motion_switcher.CheckMode(robot_form, motion_name);
  std::cout << "MotionSwitcher client API=" << motion_switcher.GetApiVersion()
            << " server API=" << motion_switcher.GetServerApiVersion()
            << " CheckMode result=" << mode_result
            << " form=\"" << robot_form << "\" name=\"" << motion_name << "\""
            << std::endl;

  unitree::robot::g1::G1ArmActionClient arm;
  arm.SetTimeout(3.0F);
  arm.Init();
  std::cout << "Arm client API=" << arm.GetApiVersion()
            << " server API=" << arm.GetServerApiVersion() << std::endl;
  std::string actions;
  const auto action_result = arm.GetActionList(actions);
  std::cout << "GetActionList result=" << action_result << " data=" << actions << std::endl;
  return service_result == 0 && action_result == 0 ? 0 : 2;
}
