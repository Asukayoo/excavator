#pragma once

#include <excavator/internal/excavator_data_type.hpp>
#include <hal/converter_interface.hpp>

namespace excavator {

class ExcavatorConverter final : public ConverterInterface {
public:
    ~ExcavatorConverter() override = default;

    bool robotCmdToRobotState(const RobotCommand& cmd, RobotState& state_out) override;
    bool robotStateToHardwareCmd(const RobotState& state, HardwareCommand& cmd_out) override;
    bool hardwareStateToRobotState(const HardwareState& raw_in, RobotState& state_out) override;

private:
    static const ExcavatorCommand* asCommand(const RobotCommand& c) {
        return dynamic_cast<const ExcavatorCommand*>(&c);
    }
    static ExcavatorState* asState(RobotState& s) { return dynamic_cast<ExcavatorState*>(&s); }
    static const ExcavatorState* asState(const RobotState& s) {
        return dynamic_cast<const ExcavatorState*>(&s);
    }
    static ExcavatorHardwareCommand* asHardwareCmd(HardwareCommand& c) {
        return dynamic_cast<ExcavatorHardwareCommand*>(&c);
    }
    static const ExcavatorHardwareState* asHardwareState(const HardwareState& raw) {
        return dynamic_cast<const ExcavatorHardwareState*>(&raw);
    }

    Vector8d resp_velocity_bias_sum_ = Vector8d::Zero();
    Vector8d resp_velocity_bias_ = Vector8d::Zero();
    std::uint32_t resp_velocity_bias_count_{0};
    bool resp_velocity_bias_ready_{false};
};

}  // namespace excavator
