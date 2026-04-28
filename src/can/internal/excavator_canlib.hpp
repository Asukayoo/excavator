#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

namespace canlib {

inline constexpr std::size_t kCanMotorFeedbackAxisCount = 8;
inline constexpr double kCanMotorRpmPlaceholder = 8190.0;

inline constexpr std::uint32_t kCanIdCmd10Hz = 0x18F021F6U;
inline constexpr std::uint32_t kCanIdCmd50HzA = 0x18F022F6U;
inline constexpr std::uint32_t kCanIdCmd50HzB = 0x18F023F6U;
inline constexpr std::uint16_t kAxisMin = 1638U;
inline constexpr std::uint16_t kAxisMax = 14742U;
inline constexpr std::size_t kExcavatorCanPayloadBytes = 8U;

struct Slave100msData {
    std::uint8_t ignition{0};
    std::uint8_t flameout{0};
    std::uint8_t crush{0};
    std::uint8_t chassis_light{0};
    std::uint8_t remote_mode{0};
    std::uint8_t pilot{0};
    std::uint8_t high_speed{0};
    std::uint8_t chassis_dozer_mode{0};
    std::uint8_t horn{0};
    std::uint8_t motor_gear{0};
    std::uint8_t estop{0};
    std::uint8_t standby{0};
    std::uint8_t remote_heartbeat{0};
};

struct Slave50HzDataA {
    std::uint16_t swing{kAxisMin};
    std::uint16_t arm{kAxisMin};
    std::uint16_t boom{kAxisMin};
    std::uint16_t bucket{kAxisMin};
};

struct Slave50HzDataB {
    std::uint16_t left_track{kAxisMin};
    std::uint16_t right_track{kAxisMin};
    std::uint16_t boom_offset{kAxisMin};
    std::uint16_t chassis_dozer{kAxisMin};
};

struct CanSharedMemoryLayout {
    std::uint64_t magic{0x42494C4E41435F32ULL};
    std::uint64_t cmd_sequence{0};
    std::uint64_t fb_sequence{0};
    Slave100msData cmd_10hz{};
    Slave50HzDataA cmd_50hz_a{};
    Slave50HzDataB cmd_50hz_b{};
    Slave100msData fb_10hz{};
    Slave50HzDataA fb_50hz_a{};
    Slave50HzDataB fb_50hz_b{};
    // 由 excavator_canlib 写入；客户端只读。暂无协议时保持 kCanMotorRpmPlaceholder（轴字语义，零速）
    std::array<double, kCanMotorFeedbackAxisCount> motor_rpm_feedback{};
};

// 反馈写入 SHM 的来源：回显指令 / 总线接收
enum class ExcavatorFeedbackSource {
    kCommandEcho,  // 以下发指令作为反馈（模拟与回环）
    kCanRx,        // 解析 RX 原始 8 字节
};

// 预留：自定义解析继承并在 start() 前 setFeedbackParser
class ExcavatorFeedbackParser {
public:
    virtual ~ExcavatorFeedbackParser() = default;
    virtual void parse10Hz(ExcavatorFeedbackSource src,
                           const std::array<std::uint8_t, kExcavatorCanPayloadBytes>& payload,
                           const Slave100msData& cmd, Slave100msData& fb_out) = 0;
    virtual void parse50HzA(ExcavatorFeedbackSource src,
                            const std::array<std::uint8_t, kExcavatorCanPayloadBytes>& payload,
                            const Slave50HzDataA& cmd, Slave50HzDataA& fb_out) = 0;
    virtual void parse50HzB(ExcavatorFeedbackSource src,
                            const std::array<std::uint8_t, kExcavatorCanPayloadBytes>& payload,
                            const Slave50HzDataB& cmd, Slave50HzDataB& fb_out) = 0;
};

// 默认：kCommandEcho 拷贝 cmd；kCanRx 使用与下发相同的 unpack 布局
class ExcavatorDefaultFeedbackParser final : public ExcavatorFeedbackParser {
public:
    void parse10Hz(ExcavatorFeedbackSource src, const std::array<std::uint8_t, kExcavatorCanPayloadBytes>& payload,
                   const Slave100msData& cmd, Slave100msData& fb_out) final;
    void parse50HzA(ExcavatorFeedbackSource src, const std::array<std::uint8_t, kExcavatorCanPayloadBytes>& payload,
                    const Slave50HzDataA& cmd, Slave50HzDataA& fb_out) final;
    void parse50HzB(ExcavatorFeedbackSource src, const std::array<std::uint8_t, kExcavatorCanPayloadBytes>& payload,
                    const Slave50HzDataB& cmd, Slave50HzDataB& fb_out) final;
};

class CanLib {
public:
    // 默认 can2 + 模拟；模拟模式不依赖真实 CAN 设备（无 can2 也可运行）
    // 真实总线模式：open() 前 setSimulationEnabled(false)
    CanLib(std::string can_if_name = "can2", std::string shm_name = "canlib_shm", bool create_mapping = false);
    ~CanLib();

    CanLib(const CanLib&) = delete;
    CanLib& operator=(const CanLib&) = delete;

    bool open();
    bool start();
    bool stop();
    bool close();
    bool isOpen() const;
    void setSimulationEnabled(bool enabled);
    bool isSimulationEnabled() const;
    void setCanBusEnabled(bool enabled);
    bool isCanBusEnabled() const;
    std::uint64_t loopTick() const;
    std::string lastError() const;
    // nullptr 表示使用 ExcavatorDefaultFeedbackParser；须在 start() 前调用
    void setFeedbackParser(std::unique_ptr<ExcavatorFeedbackParser> parser);
    ExcavatorFeedbackParser* feedbackParser();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace canlib
