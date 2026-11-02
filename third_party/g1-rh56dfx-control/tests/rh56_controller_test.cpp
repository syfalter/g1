#include "rh56/hand_controller.hpp"

#include <iostream>
#include <memory>
#include <string>

namespace {

class FakeTransport final : public rh56::Transport
{
public:
    bool ReadPosition(rh56::Position& value, uint8_t) override
    {
        value = position;
        return reads_ok;
    }

    bool ReadWords(uint16_t address, rh56::RawValues& value, uint8_t) override
    {
        value = address == 0x062E ? force : current;
        return reads_ok;
    }

    bool ReadBytes(uint16_t address, rh56::ByteValues& value, uint8_t) override
    {
        if (address == 0x0646)
            value = cleared ? rh56::ByteValues{} : error;
        else if (address == 0x064C)
            value = cleared ? rh56::ByteValues{} : status;
        else
            value = temperature;
        return reads_ok;
    }

    bool WritePosition(const rh56::Position& value,
                       const rh56::JointMask& mask, uint8_t) override
    {
        last_position = value;
        last_mask = mask;
        ++position_writes;
        return writes_ok;
    }

    bool WriteWords(uint16_t, const rh56::RawValues&, int16_t, uint8_t) override
    {
        ++word_writes;
        return writes_ok;
    }

    bool ClearErrors(uint8_t) override
    {
        ++clear_writes;
        cleared = true;
        return writes_ok;
    }

    rh56::Position position{};
    rh56::Position last_position{};
    rh56::JointMask last_mask{};
    rh56::RawValues force{};
    rh56::RawValues current{};
    rh56::ByteValues error{};
    rh56::ByteValues status{};
    rh56::ByteValues temperature{};
    bool reads_ok{true};
    bool writes_ok{true};
    bool cleared{false};
    int position_writes{0};
    int clear_writes{0};
    int word_writes{0};
};

int failures = 0;

void Check(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

void TestMaskedPositionCommand()
{
    auto transport = std::make_unique<FakeTransport>();
    auto* fake = transport.get();
    rh56::HandController controller(std::move(transport));
    rh56::Position q{};
    q[2] = 0.75f;
    const auto result = controller.SetTargets(q, uint8_t{1} << 2, 42);
    Check(static_cast<bool>(result), "valid masked command succeeds");
    Check(fake->position_writes == 1, "position is written once");
    Check(fake->last_mask[2] && !fake->last_mask[1], "joint mask is preserved");
    Check(controller.GetState().last_command_id == 42,
          "command id reaches state");

    auto feedback_transport = std::make_unique<FakeTransport>();
    feedback_transport->position[0] = 0.4f;
    rh56::HandController feedback_controller(std::move(feedback_transport));
    Check(static_cast<bool>(feedback_controller.RefreshPosition()),
          "position refresh succeeds");
    Check(feedback_controller.GetState().target_q ==
              feedback_controller.GetState().feedback_q,
          "first feedback initializes the hold target");

    q[2] = 1.1f;
    Check(controller.SetTargets(q, uint8_t{1} << 2).code ==
              rh56::ResultCode::kInvalidArgument,
          "out-of-range q is rejected");
}

void TestFaultedJointIsRejected()
{
    auto transport = std::make_unique<FakeTransport>();
    transport->error[3] = 4;
    rh56::HandController controller(std::move(transport));
    rh56::Position q{};
    Check(controller.SetTargets(q, uint8_t{1} << 3).code ==
              rh56::ResultCode::kJointFaulted,
          "faulted joint cannot move");
}

void TestFaultClear()
{
    auto transport = std::make_unique<FakeTransport>();
    auto* fake = transport.get();
    fake->error[1] = 2;
    fake->status[1] = 7;
    fake->temperature.fill(30);
    rh56::HandController controller(std::move(transport));
    const auto result = controller.ClearFault(uint8_t{1} << 1);
    Check(static_cast<bool>(result), "clearable fault succeeds");
    Check(fake->clear_writes == 1, "clear register is written once");
    Check(fake->position_writes == 2, "faulted joint is held before and after clear");
}

void TestOvertemperatureIsNotCleared()
{
    auto transport = std::make_unique<FakeTransport>();
    auto* fake = transport.get();
    fake->error[0] = 1;
    fake->temperature[0] = rh56::kMaximumSafeTemperatureC;
    rh56::HandController controller(std::move(transport));
    const auto result = controller.ClearFault(1);
    Check(result.code == rh56::ResultCode::kOverTemperature,
          "hot fault is rejected");
    Check(fake->clear_writes == 0, "hot fault never writes clear register");
}

void TestGripConfiguration()
{
    auto transport = std::make_unique<FakeTransport>();
    auto* fake = transport.get();
    fake->temperature.fill(30);
    rh56::HandController controller(std::move(transport));
    rh56::Position q{};
    q.fill(0.25f);
    const auto result = controller.ApplyGrip(q, 200, 150, 91);
    Check(static_cast<bool>(result), "safe grip succeeds");
    Check(fake->word_writes == 2, "grip writes current and force limits");
    Check(fake->position_writes == 1, "grip writes its position target");
    Check(controller.GetState().force_limit[0] == 200,
          "grip force limit reaches state");
    Check(controller.GetState().current_limit[0] == 150,
          "grip current limit reaches state");
}

void TestIdleContactDetection()
{
    auto transport = std::make_unique<FakeTransport>();
    auto* fake = transport.get();
    fake->force.fill(100);
    rh56::HandController controller(std::move(transport));
    for (int i = 0; i < 3; ++i)
        Check(static_cast<bool>(controller.RefreshForce(true, 50)),
              "idle force sample succeeds");
    Check(controller.GetState().contact_monitoring,
          "contact monitoring starts after baseline calibration");

    fake->force[3] = 170;
    controller.RefreshForce(true, 50);
    Check(controller.GetState().contact[3], "large force change detects contact");
    fake->force[3] = 110;
    controller.RefreshForce(true, 50);
    Check(!controller.GetState().contact[3], "contact clears with hysteresis");

    controller.RefreshForce(false, 50);
    Check(!controller.GetState().contact_monitoring &&
              !controller.GetState().contact[3],
          "active commands disable contact monitoring");
}

}  // namespace

int main()
{
    TestMaskedPositionCommand();
    TestFaultedJointIsRejected();
    TestFaultClear();
    TestOvertemperatureIsNotCleared();
    TestGripConfiguration();
    TestIdleContactDetection();
    if (failures == 0)
        std::cout << "rh56_controller_test: PASS\n";
    return failures == 0 ? 0 : 1;
}
