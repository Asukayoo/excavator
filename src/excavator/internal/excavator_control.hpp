#pragma once

#include <excavator/internal/excavator_data_type.hpp>
#include <hal/state_interface.hpp>

#include <array>
#include <mutex>
#include <vector>

namespace excavator {

// 控制层：单入口，根据 StateInterface 中的 mode 分支计算 ref
class ExcavatorControl final {
public:
    explicit ExcavatorControl(const StateInterface& state) : state_(state) {}
    bool setPidVectors(const std::vector<std::vector<double>>& pid_vectors);
    bool updateRef(const ExcavatorState& ref_in,
                   const ExcavatorState& resp,
                   const ExcavatorCommand* cmd_in,
                   ExcavatorState& ref_out) const;

private:
    bool openLoopMotorSpeed(const ExcavatorState& ref_in,
                            const ExcavatorState& resp,
                            const ExcavatorCommand* cmd_in,
                            ExcavatorState& ref_out) const;
    bool closedLoopJointPosition(const ExcavatorState& ref_in,
                                 const ExcavatorState& resp,
                                 const ExcavatorCommand* cmd_in,
                                 ExcavatorState& ref_out) const;
    bool closedLoopJointVelocity(const ExcavatorState& ref_in,
                                 const ExcavatorState& resp,
                                 const ExcavatorCommand* cmd_in,
                                 ExcavatorState& ref_out) const;
    bool closedLoopVelocityScalar(const ExcavatorState& ref_in,
                                  const ExcavatorState& resp,
                                  const ExcavatorCommand* cmd_in,
                                  ExcavatorState& ref_out) const;
    struct PidParams {
        Vector8d position_kp = Vector8d::Constant(35.0);
        Vector8d position_ki = Vector8d::Constant(0.8);
        Vector8d position_kd = Vector8d::Constant(4.0);
        Vector8d velocity_kp = Vector8d::Constant(35.0);
        Vector8d velocity_ki = Vector8d::Constant(0.8);
        Vector8d velocity_kd = Vector8d::Constant(4.0);
        Vector8d velocity_scalar_max = Vector8d::Constant(kPi / 20.0);
        // 标量前馈正负阈值∈[0,1]；|标量|∈(dead,1] 映射到 [|t|,1]，t 按符号取 pos/neg
        Vector8d feedforward_scalar_threshold_pos = Vector8d::Zero();
        Vector8d feedforward_scalar_threshold_neg = Vector8d::Zero();
    };
    PidParams loadPidParams() const;
    void applyZeroDriftCompensation(const ExcavatorState& resp,
                                    const Vector8d& raw_ref_scalar,
                                    Vector8d& compensated_resp_velocity,
                                    Vector8d& compensated_ref_scalar) const;

    const StateInterface& state_;
    mutable std::mutex pid_mu_{};
    mutable PidParams pid_params_{};
    mutable ExcavatorControlModeType last_mode_{ExcavatorControlModeType::OpenLoopMotorSpeed};
    mutable std::uint64_t mode_cycle_{0};
    mutable Vector8d prev_ref_velocity_{Vector8d::Zero()};
    mutable Vector8d pid_e_prev_{Vector8d::Zero()};
    mutable Vector8d pid_e_prev2_{Vector8d::Zero()};
    mutable Vector8d pid_extra_prev_{Vector8d::Zero()};
    mutable Vector8d pid_rpm_prev_{Vector8d::Constant(kMotorSpeedRawZero)};
    mutable Vector8d resp_velocity_bias_sum_{Vector8d::Zero()};
    mutable Vector8d resp_velocity_bias_{Vector8d::Zero()};
    mutable Vector8d ref_scalar_bias_sum_{Vector8d::Zero()};
    mutable Vector8d ref_scalar_bias_{Vector8d::Zero()};
    mutable std::uint32_t zero_drift_count_{0};
    mutable bool zero_drift_ready_{false};
    // 标量闭环：测量速度低通状态（不改 resp，仅反馈用）
    mutable Vector8d resp_velocity_lp_{Vector8d::Zero()};
    mutable bool resp_velocity_lp_need_init_{true};
};

}  // namespace excavator
